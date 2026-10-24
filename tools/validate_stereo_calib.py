"""Validate a Kalibr stereo calibration (pinhole-equidistant) on an ordinary stereo recording - no target needed.

Measures the angular epipolar error of SIFT matches between simultaneous left/right frames:
how far each right-camera ray lies from the epipolar plane predicted by the calibration.
Tests intrinsics + distortion + stereo rotation + baseline DIRECTION. It does NOT test baseline LENGTH (metric scale).

Usage (Windows or Linux, Python 3.9+, pip install opencv-python numpy pyyaml scipy):
  python validate_stereo_calib.py --left left.mp4 --right right.mp4 --camchain bmi088_calibration-camchain.yaml --scale 2 --out result.json
  --scale: video resolution / calibration resolution (e.g. 1600x1200 video, 800x600 calibration -> 2)
"""
import argparse,json,cv2,numpy as np,yaml
from scipy.optimize import least_squares
ap=argparse.ArgumentParser();ap.add_argument('--left',required=True);ap.add_argument('--right',required=True);ap.add_argument('--camchain',required=True)
ap.add_argument('--scale',type=float,default=1.0);ap.add_argument('--step',type=float,default=4.0,help='seconds between sampled frame pairs')
ap.add_argument('--out',default='stereo_calib_validation.json');a=ap.parse_args()
cc=yaml.safe_load(open(a.camchain))
for c in ('cam0','cam1'):
    assert cc[c]['distortion_model']=='equidistant' and cc[c]['camera_model']=='pinhole','this script handles pinhole-equidistant (Kalibr) only'
def scaled(intr,s):
    fx,fy,cx,cy=intr;off=(s-1)/2  # OpenCV pixel-centre convention: x_full = s*x_small + (s-1)/2
    return fx*s,fy*s,cx*s+off,cy*s+off
I0=scaled(cc['cam0']['intrinsics'],a.scale);I1=scaled(cc['cam1']['intrinsics'],a.scale)
k0=np.array(cc['cam0']['distortion_coeffs']);k1=np.array(cc['cam1']['distortion_coeffs'])
Tm=np.array(cc['cam1']['T_cn_cnm1']);R=Tm[:3,:3];T=Tm[:3,3]
def rays(p,I,k):
    fx,fy,cx,cy=I;x=(p[:,0]-cx)/fx;y=(p[:,1]-cy)/fy;td=np.hypot(x,y);t=td.copy()
    for _ in range(30):
        f=t*(1+k[0]*t**2+k[1]*t**4+k[2]*t**6+k[3]*t**8)-td;t-=f/(1+3*k[0]*t**2+5*k[1]*t**4+7*k[2]*t**6+9*k[3]*t**8)
    s=np.where(td>1e-9,np.sin(t)/np.maximum(td,1e-12),1.0);v=np.c_[x*s,y*s,np.cos(t)];return v/np.linalg.norm(v,axis=1,keepdims=True),np.degrees(t)
def epi(r0,r1,Rm,Tv):
    E=np.array([[0,-Tv[2],Tv[1]],[Tv[2],0,-Tv[0]],[-Tv[1],Tv[0],0]])@Rm;n=(E@r0.T).T;n/=np.linalg.norm(n,axis=1,keepdims=True)
    return np.degrees(np.arcsin(np.clip((n*r1).sum(1),-1,1)))
px_per_deg=I0[0]*np.pi/180
L=cv2.VideoCapture(a.left);Q=cv2.VideoCapture(a.right);dur=L.get(cv2.CAP_PROP_FRAME_COUNT)/L.get(cv2.CAP_PROP_FPS)
sift=cv2.SIFT_create(nfeatures=3000);bf=cv2.BFMatcher();D=[]
for sec in np.arange(a.step,dur-1,a.step):
    g=[]
    for c in (L,Q):
        c.set(cv2.CAP_PROP_POS_MSEC,sec*1000);ok,f=c.read()
        if not ok:break
        g.append(cv2.cvtColor(f,cv2.COLOR_BGR2GRAY))
    if len(g)<2:continue
    kp=[sift.detectAndCompute(x,None) for x in g];m=bf.knnMatch(kp[0][1],kp[1][1],k=2);good=[x for x,y in m if x.distance<.7*y.distance]
    if len(good)<30:continue
    p0=np.float64([kp[0][0][x.queryIdx].pt for x in good]);p1=np.float64([kp[1][0][x.trainIdx].pt for x in good])
    r0,t0=rays(p0,I0,k0);r1,_=rays(p1,I1,k1);D.append((float(sec),p0,r0,r1,t0));print(f'{sec:6.1f}s  matches {len(good)}',flush=True)
def stats(e):e=np.abs(e);return dict(n=int(len(e)),median_px=round(float(np.median(e)),3),p75_px=round(float(np.percentile(e,75)),3),p90_px=round(float(np.percentile(e,90)),3))
allerr=np.concatenate([epi(r0,r1,R,T) for _,_,r0,r1,_ in D])*px_per_deg
ang=np.concatenate([t for *_,t in D]);P=np.concatenate([p for _,p,*_ in D])
tr=D[::2];te=D[1::2]
def err(sub,d):Rd=cv2.Rodrigues(np.radians(d))[0];return np.concatenate([epi(r0,r1,Rd@R,T) for _,_,r0,r1,_ in sub])*px_per_deg
fit=least_squares(lambda d:err(tr,d),np.zeros(3),loss='soft_l1',f_scale=1.0)
W,H=int(L.get(cv2.CAP_PROP_FRAME_WIDTH)),int(L.get(cv2.CAP_PROP_FRAME_HEIGHT))
grid={}
for j in range(3):
    for i in range(3):
        s=(P[:,0]>=i*W/3)&(P[:,0]<(i+1)*W/3)&(P[:,1]>=j*H/3)&(P[:,1]<(j+1)*H/3)
        if s.sum()>20:grid[f'row{j}_col{i}']=round(float(np.median(np.abs(allerr[s]))),3)
res=dict(calibration=a.camchain,scale=a.scale,px_per_deg_at_centre=round(px_per_deg,3),pairs_sampled=len(D),
  angular_epipolar_error_all=stats(allerr),
  by_field_angle={f'{lo}-{hi}deg':stats(allerr[(ang>=lo)&(ang<hi)]) for lo,hi in [(0,30),(30,50),(50,70),(70,100)] if ((ang>=lo)&(ang<hi)).sum()>20},
  grid_3x3_median_px=grid,
  heldout_median_px_before=round(float(np.median(np.abs(err(te,np.zeros(3))))),3),
  heldout_median_px_after_3dof_fit=round(float(np.median(np.abs(err(te,fit.x)))),3),
  fitted_stereo_rotation_correction_deg=np.round(fit.x,4).tolist(),
  interpretation='Unfiltered matches (no RANSAC). Median near the matching noise floor (~0.3-0.6 px) and a fitted correction of a few hundredths of a degree indicate the calibration fits this device and sensor mode. Baseline length / metric depth are NOT tested.')
print(json.dumps(res,indent=1));json.dump(res,open(a.out,'w'),indent=1)
