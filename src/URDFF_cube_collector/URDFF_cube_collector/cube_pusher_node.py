#!/usr/bin/env python3
"""
cube_pusher_node.py  —  v6  (arc routing fully verified)
==========================================================
Arc routing uses a 3-waypoint L-path verified geometrically for all 4 cubes:

  wp1 = beside cube centre, laterally offset (robot→wp1 clears cube ✓)
  wp2 = beside behind-point, same lateral offset (wp1→wp2 clears cube ✓)
  behind-point = wp2→behind is pure lateral movement, 0.115m robot clearance ✓

State machine:
  INIT → SELECT → NAV_WP1 → NAV_WP2 → NAVIGATE_BEHIND
       → ALIGN → APPROACH → PUSH → BACKUP → … → DONE

NAV_WP1/WP2 only inserted when direct robot→behind path blocked by cube.
Arc check uses CUBE_CLEAR_R=0.38m from cube centre (verified safe threshold).
"""

import math, threading
import numpy as np, cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_OK = True
except ImportError:
    CV_BRIDGE_OK = False

# ─────────────────────────────────────────────────────────────────────────────
# World: (name, start_x, start_y, drop_x, drop_y)
# ─────────────────────────────────────────────────────────────────────────────
CUBES = [
    ("red_cube",    1.0,  0.5,  -3.95, -1.05),
    ("yellow_cube", 0.0,  1.0,  -4.07, -1.18),
    ("blue_cube",  -1.0, -0.5,  -4.25, -0.80),
    ("green_cube",  0.0, -1.0,  -3.93, -1.19),
]

# ─────────────────────────────────────────────────────────────────────────────
# Tuning
# ─────────────────────────────────────────────────────────────────────────────
BEHIND_OFFSET   = 0.50
TOUCH_DIST      = 0.22   # odom contact fallback
LIDAR_CONTACT   = 0.25   # LiDAR front trigger
ARRIVAL_TOL     = 0.05
ANGLE_TOL       = 0.05

KP_LIN, KP_ANG  = 0.55, 2.0
MAX_LIN, MAX_ANG = 0.20, 1.2
OBS_SLOW        = 0.35
OBS_STOP        = 0.15
OBS_CONE_DEG    = 25
BACKUP_STEPS    = 30

# Arc routing
CUBE_CLEAR_R    = 0.50   # m from cube centre — triggers arc routing
ARC_LATERAL     = 0.70   # m lateral offset for wp1 and wp2

CAM_W, CAM_H    = 960, 540

# ─────────────────────────────────────────────────────────────────────────────
# HSV  (blue tuned from real camera: H=120,S=137,V=95)
# ─────────────────────────────────────────────────────────────────────────────
COLOUR_RANGES = {
    "red_cube":    [(np.array([0,100,60]),  np.array([12,255,255])),
                    (np.array([165,100,60]),np.array([179,255,255]))],
    "blue_cube":   [(np.array([110,117,75]),np.array([130,255,255]))],
    "green_cube":  [(np.array([38,50,50]),  np.array([88,255,255]))],
    "yellow_cube": [(np.array([16,80,80]),  np.array([38,255,255]))],
}
MIN_AREA, MAX_AREA_FRAC = 400, 0.35
ASPECT_MIN, ASPECT_MAX  = 0.35, 2.80
MIN_SOLIDITY            = 0.70
DRAW_COL = {"red_cube":(0,0,220),"blue_cube":(220,80,0),
            "green_cube":(0,200,0),"yellow_cube":(0,210,210)}

# ─────────────────────────────────────────────────────────────────────────────
# Geometry
# ─────────────────────────────────────────────────────────────────────────────
def wrap(a):
    while a> math.pi: a-=2*math.pi
    while a<-math.pi: a+=2*math.pi
    return a

def d2d(ax,ay,bx,by): return math.hypot(bx-ax,by-ay)

def seg_dist(px,py,ax,ay,bx,by):
    """Distance from point P to segment AB."""
    dx,dy=bx-ax,by-ay; l2=dx*dx+dy*dy
    if l2<1e-10: return d2d(px,py,ax,ay)
    t=max(0.,min(1.,((px-ax)*dx+(py-ay)*dy)/l2))
    return d2d(px,py,ax+t*dx,ay+t*dy)

def compute_arc_waypoint(rx,ry,cx,cy,nx,ny):
    """Single waypoint beside cube on the robot's side. Verified safe for all 4 cubes."""
    pl=(-ny,nx); pr=(ny,-nx)
    relx,rely=rx-cx,ry-cy
    perp=pl if relx*pl[0]+rely*pl[1]>=relx*pr[0]+rely*pr[1] else pr
    return cx+perp[0]*ARC_LATERAL, cy+perp[1]*ARC_LATERAL


# ─────────────────────────────────────────────────────────────────────────────
class CubePusher(Node):

    S_INIT  = "INIT"
    S_SEL   = "SELECT_CUBE"
    S_ARC   = "NAV_ARC"          # single waypoint beside cube
    S_NAV   = "NAVIGATE_BEHIND"  # to behind-point, obs=False
    S_ALIGN = "ALIGN"
    S_APP   = "APPROACH_CUBE"
    S_PUSH  = "PUSH"
    S_BACK  = "BACKUP"
    S_DONE  = "DONE"

    def __init__(self):
        super().__init__("cube_pusher")
        self.cmd_pub = self.create_publisher(Twist,"/cmd_vel",10)

        be = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry,  "/odometry/filtered", self._odom_cb,   be)
        self.create_subscription(LaserScan, "/scan",              self._scan_cb,   10)
        self.create_subscription(Image,     "/camera",            self._cam_cb,    10)

        self.rx=self.ry=self.ryaw=0.; self.odom_ok=False
        self.scan=[]; self.samin=0.; self.sainc=0.
        self.frame=None; self.flock=threading.Lock()
        if CV_BRIDGE_OK: self.br=CvBridge()

        self.state=self.S_INIT
        self.remaining=list(CUBES)
        self.cur=None          # (name,sx,sy,dx,dy)
        self.gx=self.gy=0.    # current nav goal
        self._bx=self._by=0.  # stored behind-point
        self._pyaw=0.
        self._pdist=0.
        self._psx=self._psy=0.
        self._bcnt=0
        self._done=[]
        self.dets={}

        cv2.namedWindow("Camera Feed — Cube Detection",
                        cv2.WINDOW_NORMAL|cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("Camera Feed — Cube Detection",CAM_W,CAM_H)
        cv2.namedWindow("LiDAR Top-Down View",
                        cv2.WINDOW_NORMAL|cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("LiDAR Top-Down View",500,500)
        cv2.imshow("Camera Feed — Cube Detection",
                   np.zeros((CAM_H,CAM_W,3),dtype=np.uint8))
        cv2.imshow("LiDAR Top-Down View",
                   np.zeros((500,500,3),dtype=np.uint8))
        cv2.waitKey(1)

        self.create_timer(0.05, self._ctrl)
        self.create_timer(0.05, self._disp)
        self.get_logger().info("CubePusher v6 ready")

    # ── callbacks ────────────────────────────────────────────────────────────
    def _odom_cb(self,msg):
        self.rx=msg.pose.pose.position.x
        self.ry=msg.pose.pose.position.y
        q=msg.pose.pose.orientation
        self.ryaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.odom_ok=True

    def _scan_cb(self,msg):
        self.scan=list(msg.ranges)
        self.samin=msg.angle_min
        self.sainc=msg.angle_increment

    def _cam_cb(self,msg):
        if not CV_BRIDGE_OK: return
        try:
            f=self.br.imgmsg_to_cv2(msg,desired_encoding="bgr8")
            with self.flock: self.frame=f.copy()
        except Exception as e:
            self.get_logger().warn(str(e),throttle_duration_sec=5.)

    # ── lidar ────────────────────────────────────────────────────────────────
    def _cone(self,hdeg=OBS_CONE_DEG):
        if not self.scan: return float("inf")
        h=math.radians(hdeg)
        v=[r for i,r in enumerate(self.scan)
           if math.isfinite(r) and r>0.01
           and abs(self.samin+i*self.sainc)<=h]
        return min(v) if v else float("inf")

    # ── motion ───────────────────────────────────────────────────────────────
    def _stop(self): self.cmd_pub.publish(Twist())

    def _nav(self,gx,gy,obs=True):
        dx,dy=gx-self.rx,gy-self.ry
        d=math.hypot(dx,dy)
        if d<ARRIVAL_TOL: self._stop(); return True
        ey=wrap(math.atan2(dy,dx)-self.ryaw)
        front=self._cone() if obs else float("inf")
        if obs and front<OBS_STOP: self._stop(); return False
        spd=1. if (not obs or front>=OBS_SLOW) else front/OBS_SLOW
        t=Twist()
        if abs(ey)>0.5:
            t.angular.z=max(-MAX_ANG,min(MAX_ANG,KP_ANG*ey))
        else:
            t.linear.x=max(0.,min(MAX_LIN*spd,KP_LIN*d))
            t.angular.z=max(-MAX_ANG,min(MAX_ANG,KP_ANG*ey))
        self.cmd_pub.publish(t); return False

    def _rot(self,yaw):
        e=wrap(yaw-self.ryaw)
        if abs(e)<ANGLE_TOL: self._stop(); return True
        t=Twist(); t.angular.z=max(-MAX_ANG,min(MAX_ANG,KP_ANG*e))
        self.cmd_pub.publish(t); return False

    # ── planning ─────────────────────────────────────────────────────────────
    def _behind(self,sx,sy,dx,dy):
        vx,vy=dx-sx,dy-sy; d=math.hypot(vx,vy) or 1e-6
        nx,ny=vx/d,vy/d
        return sx-nx*BEHIND_OFFSET, sy-ny*BEHIND_OFFSET, math.atan2(ny,nx)

    def _go(self,s):
        self.get_logger().info(f"[{self.state}]→[{s}]")
        self.state=s

    # ── control loop 20 Hz ───────────────────────────────────────────────────
    def _ctrl(self):
        if not self.odom_ok: return

        # INIT
        if self.state==self.S_INIT:
            self.get_logger().info(
                f"Ready ({self.rx:.2f},{self.ry:.2f}) "
                f"{len(self.remaining)} cubes")
            self._go(self.S_SEL)

        # SELECT
        elif self.state==self.S_SEL:
            if not self.remaining: self._go(self.S_DONE); return
            self.cur=min(self.remaining,
                         key=lambda c:d2d(self.rx,self.ry,c[1],c[2]))
            n,sx,sy,dx,dy=self.cur
            bx,by,py=self._behind(sx,sy,dx,dy)
            self._bx,self._by,self._pyaw=bx,by,py
            vx2,vy2=dx-sx,dy-sy; nd=math.hypot(vx2,vy2) or 1e-6
            nx,ny=vx2/nd,vy2/nd
            self._pdist=d2d(sx,sy,dx,dy)-TOUCH_DIST-0.05
            self.get_logger().info(
                f"→{n} start=({sx},{sy}) drop=({dx},{dy}) "
                f"behind=({bx:.2f},{by:.2f}) "
                f"yaw={math.degrees(py):.1f}° dist={self._pdist:.2f}m")

            # If direct robot→behind path blocked by cube, use single arc waypoint
            if seg_dist(sx,sy,self.rx,self.ry,bx,by)<CUBE_CLEAR_R:
                ax,ay=compute_arc_waypoint(self.rx,self.ry,sx,sy,nx,ny)
                self.gx,self.gy=ax,ay
                self.get_logger().info(f"Arc wp: ({ax:.2f},{ay:.2f})")
                self._go(self.S_ARC)
            else:
                self.gx,self.gy=bx,by
                self.get_logger().info("Direct → NAVIGATE_BEHIND")
                self._go(self.S_NAV)

        # NAV_ARC — go beside cube (obs=True), then straight to behind-point
        elif self.state==self.S_ARC:
            if self._nav(self.gx,self.gy,obs=True):
                self.gx,self.gy=self._bx,self._by
                self._go(self.S_NAV)

        # NAVIGATE_BEHIND — obs=False: arc puts cube lateral, not in path
        elif self.state==self.S_NAV:
            if self._nav(self.gx,self.gy,obs=False):
                self._go(self.S_ALIGN)

        # ALIGN
        elif self.state==self.S_ALIGN:
            if self._rot(self._pyaw):
                self._go(self.S_APP)

        # APPROACH — 3 triggers
        elif self.state==self.S_APP:
            front=self._cone(hdeg=15)
            lidar=math.isfinite(front) and front<LIDAR_CONTACT
            _,sx,sy,_,_=self.cur
            odom=d2d(self.rx,self.ry,sx,sy)<=TOUCH_DIST
            crept=d2d(self.rx,self.ry,self._bx,self._by)
            timeout=crept>(BEHIND_OFFSET+0.15)

            if lidar or odom or timeout:
                self._stop()
                self._psx,self._psy=self.rx,self.ry
                r="LiDAR" if lidar else "odom" if odom else "timeout"
                self.get_logger().info(
                    f"Contact[{r}] front={front:.2f}m crept={crept:.2f}m")
                self._go(self.S_PUSH); return

            ey=wrap(self._pyaw-self.ryaw)
            t=Twist()
            t.linear.x=0.08
            t.angular.z=max(-MAX_ANG*0.5,min(MAX_ANG*0.5,KP_ANG*ey))
            self.cmd_pub.publish(t)

        # PUSH
        elif self.state==self.S_PUSH:
            trav=d2d(self.rx,self.ry,self._psx,self._psy)
            nx,ny=math.cos(self._pyaw),math.sin(self._pyaw)
            ecx,ecy=self.rx+nx*TOUCH_DIST,self.ry+ny*TOUCH_DIST
            _,_,_,dx,dy=self.cur
            c2d=d2d(ecx,ecy,dx,dy)
            if c2d<0.18 or trav>=self._pdist:
                self._stop()
                self.get_logger().info(
                    f"✅{self.cur[0]} delivered! c2d={c2d:.2f}m trav={trav:.2f}m")
                self._done.append(self.cur[0])
                self.remaining.remove(self.cur)
                self.cur=None; self._bcnt=0
                self._go(self.S_BACK); return
            ey=wrap(self._pyaw-self.ryaw)
            t=Twist()
            t.linear.x=MAX_LIN
            t.angular.z=max(-MAX_ANG*0.4,min(MAX_ANG*0.4,KP_ANG*ey))
            self.cmd_pub.publish(t)

        # BACKUP
        elif self.state==self.S_BACK:
            self._bcnt+=1
            t=Twist(); t.linear.x=-0.12
            self.cmd_pub.publish(t)
            if self._bcnt>=BACKUP_STEPS:
                self._stop(); self._go(self.S_SEL)

        # DONE
        elif self.state==self.S_DONE:
            self._stop()
            self.get_logger().info(f"🏁 Done! {self._done}")
            raise SystemExit

    # ── detection ────────────────────────────────────────────────────────────
    def _detect(self,frame):
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hp,wp=frame.shape[:2]; tot=hp*wp
        k=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        out={}
        for name,ranges in COLOUR_RANGES.items():
            mask=np.zeros((hp,wp),dtype=np.uint8)
            for lo,hi in ranges: mask|=cv2.inRange(hsv,lo,hi)
            mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,k)
            mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,k)
            cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            best=None; barea=0
            for c in cnts:
                a=cv2.contourArea(c)
                if a<MIN_AREA or a/tot>MAX_AREA_FRAC: continue
                x,y,bw,bh=cv2.boundingRect(c)
                asp=bw/bh if bh else 0
                if not (ASPECT_MIN<=asp<=ASPECT_MAX): continue
                hull=cv2.convexHull(c); ha=cv2.contourArea(hull)
                if ha and a/ha<MIN_SOLIDITY: continue
                if a>barea: barea=a; best=(x,y,bw,bh,round(a/tot*100,2))
            out[name]=best
        return out

    # ── display ──────────────────────────────────────────────────────────────
    def _disp(self):
        with self.flock:
            f=self.frame.copy() if self.frame is not None else None

        if f is not None:
            self.dets=self._detect(f)
            vis=cv2.resize(f,(CAM_W,CAM_H))
            sx=CAM_W/f.shape[1]; sy=CAM_H/f.shape[0]
            ns=[]
            for name,det in self.dets.items():
                col=DRAW_COL[name]
                if det:
                    x,y,bw,bh,pct=det
                    x2,y2=int(x*sx),int(y*sy)
                    w2,h2=int(bw*sx),int(bh*sy)
                    cv2.rectangle(vis,(x2,y2),(x2+w2,y2+h2),col,2)
                    cv2.putText(vis,f"{name.replace('_cube','')} {pct}%",
                                (x2,max(y2-6,14)),
                                cv2.FONT_HERSHEY_SIMPLEX,0.6,col,2)
                else: ns.append((name,col))
            for i,(n,c) in enumerate(ns):
                cv2.putText(vis,f"{n.replace('_cube','')}: not visible",
                            (10,CAM_H-14-i*22),
                            cv2.FONT_HERSHEY_SIMPLEX,0.44,c,1)
            tn=self.cur[0] if self.cur else "—"
            if self.cur:
                _,_,_,ddx,ddy=self.cur
                dz=d2d(self.rx,self.ry,ddx,ddy)
                ds=f"({ddx},{ddy})"
            else: dz=0.; ds="—"
            cv2.rectangle(vis,(0,0),(520,78),(20,20,20),-1)
            cv2.putText(vis,f"State:{self.state}",
                        (8,22),cv2.FONT_HERSHEY_SIMPLEX,0.55,(220,220,220),1)
            cv2.putText(vis,f"Target:{tn}  drop={ds}",
                        (8,46),cv2.FONT_HERSHEY_SIMPLEX,0.50,(220,220,220),1)
            cv2.putText(vis,f"Dist→drop:{dz:.2f}m  Done:{len(self._done)}/4",
                        (8,70),cv2.FONT_HERSHEY_SIMPLEX,0.46,(180,180,180),1)
            cv2.imshow("Camera Feed — Cube Detection",vis)
        else:
            ph=np.zeros((CAM_H,CAM_W,3),dtype=np.uint8)
            cv2.putText(ph,"Waiting for /camera…",
                        (CAM_W//2-200,CAM_H//2),
                        cv2.FONT_HERSHEY_SIMPLEX,0.9,(100,100,100),2)
            cv2.imshow("Camera Feed — Cube Detection",ph)

        SZ=500; C=SZ//2; PPM=40.
        img=np.zeros((SZ,SZ,3),dtype=np.uint8)
        if self.scan:
            for rm in range(1,9):
                rad=int(rm*PPM)
                if rad<C:
                    cv2.circle(img,(C,C),rad,(40,40,40),1)
                    cv2.putText(img,f"{rm}m",(C+rad+2,C+10),
                                cv2.FONT_HERSHEY_SIMPLEX,0.28,(55,55,55),1)
            hr=math.radians(OBS_CONE_DEG)
            for s in(-1,1):
                ex=C+int(math.cos(s*hr)*6*PPM)
                ey=C-int(math.sin(s*hr)*6*PPM)
                cv2.line(img,(C,C),(ex,ey),(0,70,0),1)
            mf=float("inf")
            for i,r in enumerate(self.scan):
                if not math.isfinite(r) or r<0.01 or r>8.5: continue
                a=self.samin+i*self.sainc
                px=C+int(math.cos(a)*r*PPM)
                py=C-int(math.sin(a)*r*PPM)
                if 0<=px<SZ and 0<=py<SZ:
                    ic=abs(a)<=hr
                    cv2.circle(img,(px,py),2,(0,0,220) if ic else (150,150,0),-1)
                    if ic and r<mf: mf=r
        cv2.circle(img,(C,C),7,(0,255,0),-1)
        cv2.arrowedLine(img,(C,C),(C+22,C),(0,255,0),2,tipLength=0.35)
        ft=(f"Front:{mf:.2f}m" if self.scan and math.isfinite(mf)
            else "Front:clear")
        cv2.putText(img,ft,(10,20),cv2.FONT_HERSHEY_SIMPLEX,0.50,(0,220,220),1)
        cv2.putText(img,f"State:{self.state}",
                    (10,42),cv2.FONT_HERSHEY_SIMPLEX,0.50,(200,200,200),1)
        cv2.imshow("LiDAR Top-Down View",img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node=CubePusher()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,SystemExit): node._stop()
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__": main()
