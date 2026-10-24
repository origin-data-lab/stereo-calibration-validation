"""Check where OpenCV's cv2.fisheye.undistortPoints stops being exact for YOUR equidistant (Kalibr / OpenCV fisheye) coefficients.

In the equidistant model the distorted angle is theta_d = theta * (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8).
OpenCV clamps theta_d at pi/2 when un-projecting, so with positive k1 every point whose true ray angle maps beyond
theta_d = pi/2 comes back at the wrong angle. This script projects rays at known angles, un-projects them with OpenCV
and with a Newton inversion of theta_d(theta), and prints both errors.

Usage:
  python fisheye_roundtrip_check.py --k 0.12 0.0 0.0 0.0 --f 480 --cx 800 --cy 600
  python fisheye_roundtrip_check.py --camchain camchain.yaml --cam cam0 --scale 2
"""
import argparse,numpy as np,cv2
ap=argparse.ArgumentParser();ap.add_argument('--k',type=float,nargs=4);ap.add_argument('--f',type=float,default=500.0)
ap.add_argument('--cx',type=float,default=800.0);ap.add_argument('--cy',type=float,default=600.0)
ap.add_argument('--camchain');ap.add_argument('--cam',default='cam0');ap.add_argument('--scale',type=float,default=1.0);a=ap.parse_args()
if a.camchain:
    import yaml;c=yaml.safe_load(open(a.camchain))[a.cam];fx,fy,cx,cy=c['intrinsics'];s=a.scale;off=(s-1)/2
    k=np.array(c['distortion_coeffs'],float);K=np.array([[fx*s,0,cx*s+off],[0,fy*s,cy*s+off],[0,0,1.]])
else:
    k=np.array(a.k,float);K=np.array([[a.f,0,a.cx],[0,a.f,a.cy],[0,0,1.]])
def theta_d(t):return t*(1+k[0]*t**2+k[1]*t**4+k[2]*t**6+k[3]*t**8)
def newton(td):
    t=td.copy()
    for _ in range(30):
        t-= (theta_d(t)-td)/(1+3*k[0]*t**2+5*k[1]*t**4+7*k[2]*t**6+9*k[3]*t**8)
    return t
deg=np.arange(30,91,1.0);th=np.radians(deg)
P=np.c_[np.tan(th)*np.cos(.7),np.tan(th)*np.sin(.7),np.ones(len(th))]
pix=cv2.fisheye.projectPoints(P.reshape(-1,1,3),np.zeros(3),np.zeros(3),K,k.reshape(4,1))[0].reshape(-1,2)
ocv=cv2.fisheye.undistortPoints(pix.reshape(-1,1,2),K,k.reshape(4,1)).reshape(-1,2)
ocv_deg=np.degrees(np.arctan(np.hypot(ocv[:,0],ocv[:,1])))
x=(pix[:,0]-K[0,2])/K[0,0];y=(pix[:,1]-K[1,2])/K[1,1];new_deg=np.degrees(newton(np.hypot(x,y)))
lim=None
for t in np.linspace(0,np.pi/2,20001):
    if theta_d(t)>=np.pi/2:lim=np.degrees(t);break
print(f'OpenCV {cv2.__version__}; k = {k.tolist()}')
print('theta_d reaches pi/2 at true angle:',f'{lim:.1f} deg' if lim else 'never (within 90 deg)')
print(f'{"angle":>6} {"radius_px":>10} {"opencv_err_deg":>15} {"newton_err_deg":>15}')
for d,p,o,n in zip(deg,pix,ocv_deg,new_deg):
    if d%5==0 or (lim and lim-3<=d<=lim+3):
        print(f'{d:6.0f} {np.hypot(*(p-K[:2,2])):10.0f} {d-o:15.3f} {d-n:15.2e}')
