"""Shared path, build, ROS-master and plotting support for this evidence bundle."""
from pathlib import Path
import contextlib,hashlib,json,os,shlex,shutil,signal,subprocess,time
PACKAGE=Path(__file__).resolve().parent
REPO=PACKAGE.parent

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def fresh(path):
    path=Path(path).resolve()
    if path.exists() and any(path.iterdir()):raise RuntimeError('Use a new empty output directory: '+str(path))
    path.mkdir(parents=True,exist_ok=True)
    return path

def copy_inputs(source,dest):
    dest.mkdir(parents=True,exist_ok=True)
    for p in Path(source).glob('val_*_*.csv'):
        if p.name.endswith(('_path.csv','_states.csv','_geometry.csv','_summary.csv')):shutil.copyfile(p,dest/p.name)

def build(source_specs,out):
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=True)
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','roscpp','tf'],text=True))
    reports=[]
    for name,code in source_specs:
        code=Path(code).resolve();binary=out/name
        command=[os.environ.get('CXX','g++'),'-std=c++14','-O2','-I'+str(code.parent),'-I/usr/include/eigen3',str(code),'-o',str(binary)]+flags
        subprocess.run(command,check=True)
        reports.append({'name':name,'command':command,'code_sha256':sha(code),'binary_sha256':sha(binary)})
    (out/'build.json').write_text(json.dumps(reports,indent=2)+'\n')

@contextlib.contextmanager
def master(out,port,extra=None):
    out=Path(out);env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_LOG_DIR=str(out/'roslog'))
    env.update(extra or {})
    for command in ['roscore','rosparam']:
        if not shutil.which(command):raise RuntimeError('Source the ROS Noetic environment before running: missing '+command)
    if subprocess.run(['rosparam','list'],env=env,capture_output=True).returncode==0:raise RuntimeError('Requested private master port is already in use')
    with (out/'master.log').open('w') as log:
        process=subprocess.Popen(['roscore','-p',str(port)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            for _ in range(60):
                if subprocess.run(['rosparam','list'],env=env,capture_output=True).returncode==0:break
                time.sleep(.2)
            else:raise RuntimeError('Private ROS master startup failed')
            yield env
        finally:
            os.killpg(process.pid,signal.SIGINT);process.wait(timeout=20)

def register_times(font_dir=None):
    from matplotlib import font_manager
    if font_dir:
        fonts=[p for p in Path(font_dir).iterdir() if p.suffix.lower()=='.ttf']
        if not fonts:raise FileNotFoundError('No TTF fonts in '+str(font_dir))
        if hasattr(font_manager.fontManager,'addfont'):
            for p in fonts:font_manager.fontManager.addfont(str(p))
        else:font_manager.fontManager.ttflist.extend(font_manager.createFontList([str(p) for p in fonts]))
    font_manager.findfont('Times New Roman',fallback_to_default=False)
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Times New Roman','pdf.fonttype':42,'ps.fonttype':42})
