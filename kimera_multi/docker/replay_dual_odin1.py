"""Launch two isolated ROS 1 frontends concurrently on this Docker host."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for slot in ['a','b']:
        parser.add_argument('--bag-'+slot,type=Path,required=True)
        parser.add_argument('--params-'+slot,type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rate',type=float,default=1)
    parser.add_argument('--frontend',choices=['sdk','vio'],default='sdk')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    def container_path(path):
        try:
            return '/workspace/'+str(path.resolve().relative_to(root))
        except ValueError:
            parser.error('All paths must be inside the mounted repository')
    output=container_path(args.output)
    commands={}
    for index,slot in enumerate(['a','b']):
        bag,params=getattr(args,'bag_'+slot),getattr(args,'params_'+slot)
        if not bag.is_file() or not (params/'PipelineParams.yaml').is_file():
            parser.error('Missing bag or generated parameters for '+slot)
        commands[slot]=[str(root/'docker/odin1.bash'),'replay','--bag',container_path(bag),
                       '--params',container_path(params),'--robot-name','odin_'+slot,
                       '--robot-id',str(index),'--frontend',args.frontend,
                       '--rate',str(args.rate),'--output',output+'/odin_'+slot]
    args.output.mkdir(parents=True,exist_ok=False)
    jobs,logs,timing={},[],{}
    error=None
    try:
        for slot,command in commands.items():
            log=(args.output/('odin_'+slot+'.log')).open('w');logs.append(log)
            timing[slot]={'started_unix_s':time.time(),'command':command}
            jobs[slot]=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        while any(p.poll() is None for p in jobs.values()):
            for slot,p in jobs.items():
                if p.poll() is not None and 'ended_unix_s' not in timing[slot]:
                    timing[slot]['ended_unix_s']=time.time()
            time.sleep(.2)
    except (Exception,KeyboardInterrupt) as exc:
        error=str(exc) or type(exc).__name__
    finally:
        for slot,p in jobs.items():
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=15)
            timing[slot].setdefault('ended_unix_s',time.time())
            timing[slot]['exit_code']=p.returncode
        for log in logs:log.close()
        summaries={}
        for slot in jobs:
            path=args.output/('odin_'+slot)/'summary.json'
            if path.is_file():
                d=json.loads(path.read_text())
                summaries[slot]={k:v for k,v in d.items() if k not in
                                 ['odometry','reference_odometry','parameter_sha256','source_revisions']}
        success=(error is None and len(summaries)==2 and
                 all(p.returncode==0 for p in jobs.values()) and
                 all(s.get('success') for s in summaries.values()))
        result=dict(success=success,error=error,rate=args.rate,frontend=args.frontend,runs=timing,summaries=summaries,
                    scope='Two concurrent independent '+args.frontend+' frontends with robot IDs 0 and 1. No inter-robot loop closure or distributed optimization is launched.',
                    orchestrator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    return 0 if success else 1


if __name__=='__main__':
    raise SystemExit(main())
