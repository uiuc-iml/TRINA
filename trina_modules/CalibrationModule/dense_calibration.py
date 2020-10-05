from klampt.math import se3,so3
from klampt.math import vectorops
from klampt.vis import GLProgram,camera,gldraw
import random,sys,os,math,numpy as np
import cv2,cv2.aruco as aruco
from klampt import WorldModel
import OpenGL.GL as gl

class GLVisualizer(GLProgram):
    def __init__(self,world,robot,sim):
        GLProgram.__init__(self,"Visualizer")
        self.zeroZ=True
        self.world=world
        self.robot=robot
        self.sim=sim
        self.dt=1/60.0
        self.init_camera()

    def look_at(self,pos,tgt,scale=None):
        cam=self.view.camera
        if scale is not None:
            cam.dist=vectorops.norm(vectorops.sub(tgt,pos))*scale
        cam.rot=self.get_camera_rot(vectorops.sub(pos,tgt))
        cam.tgt=tgt
        
    def get_camera_pos(self):
        cam=self.view.camera
        z=math.sin(-cam.rot[1])
        x=math.sin(cam.rot[2])*math.cos(cam.rot[1])
        y=math.cos(cam.rot[2])*math.cos(cam.rot[1])
        pos=[x,y,z]
        return vectorops.add(cam.tgt,vectorops.mul(pos,cam.dist))
    
    def get_camera_rot(self,d):
        angz=math.atan2(d[0],d[1])
        angy=math.atan2(-d[2],math.sqrt(d[0]*d[0]+d[1]*d[1]))
        return [0,angy,angz]

    def get_camera_dir(self,zeroZ=False):
        cam=self.view.camera
        dir=vectorops.sub(cam.tgt,self.get_camera_pos())
        if zeroZ:
            dir=(dir[0],dir[1],0)
        if vectorops.norm(dir)>1e-6:
            dir=vectorops.mul(dir,1/vectorops.norm(dir))
        dir[1]*=-1
        return dir

    def get_left_dir(self,zeroZ=False):
        dir=vectorops.cross([0,0,1],self.get_camera_dir())
        if zeroZ:
            dir=(dir[0],dir[1],0)
        if vectorops.norm(dir)>1e-6:
            dir=vectorops.mul(dir,1/vectorops.norm(dir))
        return dir

    def init_camera(self):
        if self.robot is None:
            bb=((-1.,-1.,-1.),(1.,1.,1.))
        else: bb=self.get_robot_bb()
        pos=[bb[1][0],(bb[0][1]+bb[1][1])/2,bb[1][2]]
        tgt=[bb[0][0],(bb[0][1]+bb[1][1])/2,bb[0][2]]
        self.look_at(pos,tgt,2.0)
        self.moveSpd=0.005
        self.zoomSpd=1.03
        self.zoomMin=0.01
        self.zoomMax=100.0
        
        self.zoomInCam=False
        self.zoomOutCam=False
        self.forwardCam=False
        self.backCam=False
        self.leftCam=False
        self.rightCam=False
        self.raiseCam=False
        self.sinkCam=False
        return
 
    def get_link_children(self,linkId):
        return [i for i in range(self.robot.numLinks()) if self.robot.link(i).getParent()==linkId]

    def get_robot_root_link(self):
        for i in range(self.robot.numLinks()):
            if str(self.robot.link(i).geometry().type())!='':
                return i,self.robot.link(i)

    def get_robot_bb(self,link0=None):
        bb=None
        if link0 is None:
            link0=self.get_robot_root_link()[0]
        bb=self.robot.link(link0).geometry().getBB()
        for c in self.get_link_children(link0):
            bb=GLVisualizer.union_bb(bb,self.get_robot_bb(c))
        return bb

    @staticmethod
    def union_bb(a,b):
        if isinstance(b,list):
            return union_bb(a,(b,b))
        else:
            return ([min(c[0],c[1]) for c in zip(a[0],b[0])],   \
                    [max(c[0],c[1]) for c in zip(a[1],b[1])])

    def keyboardfunc(self,c,x,y):
        if c==b'f':
            self.init_camera()
        elif c==b'z':
            self.zeroZ=not self.zeroZ
        elif c==b'q':
            self.zoomInCam=True
        elif c==b'e':
            self.zoomOutCam=True
        elif c==b'w':
            self.forwardCam=True
        elif c==b's':
            self.backCam=True
        elif c==b'a':
            self.leftCam=True
        elif c==b'd':
            self.rightCam=True
        elif c==b' ':
            self.raiseCam=True
        elif c==b'c':
            self.sinkCam=True
        elif c==b',':
            import pickle
            cam=self.view.camera
            pickle.dump((cam.dist,self.get_camera_pos(),cam.tgt),open(DATA_PATH+"/tmpCamera.dat",'wb'))
            print('Saved camera to tmpCamera.dat!')
        elif c==b'.':
            import pickle
            cam=self.view.camera
            cam.dist,pos,tgt=pickle.load(open(DATA_PATH+"/tmpCamera.dat",'rb'))
            self.look_at(pos,tgt)
            print('Loaded camera to tmpCamera.dat!')

    def keyboardupfunc(self,c,x,y):
        if c==b'q':
            self.zoomInCam=False
        elif c==b'e':
            self.zoomOutCam=False
        elif c==b'w':
            self.forwardCam=False
        elif c==b's':
            self.backCam=False
        elif c==b'a':
            self.leftCam=False
        elif c==b'd':
            self.rightCam=False
        elif c==b' ':
            self.raiseCam=False
        elif c==b'c':
            self.sinkCam=False

    def display(self):
        if self.sim is not None:
            self.sim.updateWorld()
        self.world.drawGL()

    def handle_camera(self):
        self.view.clippingplanes=(self.view.clippingplanes[0],self.zoomMax)
        cam=self.view.camera
        moveSpd=self.moveSpd*cam.dist
        if self.zoomInCam:
            cam.dist=max(cam.dist/self.zoomSpd,self.zoomMin)
        elif self.zoomOutCam:
            cam.dist=min(cam.dist*self.zoomSpd,self.zoomMax)
        elif self.forwardCam:
            delta=vectorops.mul(self.get_camera_dir(self.zeroZ),moveSpd)
            self.look_at(vectorops.add(self.get_camera_pos(),delta),vectorops.add(cam.tgt,delta))
        elif self.backCam:
            delta=vectorops.mul(self.get_camera_dir(self.zeroZ),-moveSpd)
            self.look_at(vectorops.add(self.get_camera_pos(),delta),vectorops.add(cam.tgt,delta))
        elif self.leftCam:
            delta=vectorops.mul(self.get_left_dir(self.zeroZ),moveSpd)
            self.look_at(vectorops.add(self.get_camera_pos(),delta),vectorops.add(cam.tgt,delta))
        elif self.rightCam:
            delta=vectorops.mul(self.get_left_dir(self.zeroZ),-moveSpd)
            self.look_at(vectorops.add(self.get_camera_pos(),delta),vectorops.add(cam.tgt,delta))
        elif self.raiseCam:
            delta=(0,0,moveSpd)
            self.look_at(vectorops.add(self.get_camera_pos(),delta),vectorops.add(cam.tgt,delta))
        elif self.sinkCam:
            delta=(0,0,-moveSpd)
            self.look_at(vectorops.add(self.get_camera_pos(),delta),vectorops.add(cam.tgt,delta))

    def idle(self):
        self.handle_camera()
        if self.sim is not None:
            self.sim.simulate(self.dt)
        self.update_animation()
    
    def render(self,fn,tfn):
        pass
    
    def update_animation(self):
        if not hasattr(self,"animation_fn") or self.animation_fn is None:
            return
        if hasattr(self,'render_path') and self.render_path is not None:
            tfn=self.render_path+"/frm%d.pov"%len(self.animation_frms)
            cmd=self.render(None,tfn)
            continueAnim=self.animation_op()
            if continueAnim:
                self.animation_frms.append(cmd)
            else:
                import pickle
                pickle.dump(self.animation_frms,open(self.render_path+"/cmd.dat",'wb'))
                print("Finishing saving animation to %s!"%self.render_path)
                self.animation_fn=None
                self.animation_op=None
                self.animation_dur=None
                self.animation_frms=None
        else:
            import cv2
            tmpPath="tmp.png"
            continueAnim=self.animation_op()
            self.save_screen(tmpPath,multithreaded=False)
            if continueAnim:
                self.animation_frms.append(cv2.imread(tmpPath))
            else:
                fourcc=cv2.cv.CV_FOURCC(*'XVID')
                height,width,layers=self.animation_frms[0].shape
                out=cv2.VideoWriter(self.animation_fn,fourcc,1.0/self.animation_dur,(width,height))
                for f in self.animation_frms:
                    out.write(f)
                out.release()
                if os.path.exists(tmpPath):
                    os.remove(tmpPath)
                print("Finishing saving animation to %s!"%self.animation_fn)
                self.animation_fn=None
                self.animation_op=None
                self.animation_dur=None
                self.animation_frms=None
    
    def render_animation(self,fn,op,dur=1,render_path=None):
        if hasattr(self,"animation_fn") and self.animation_fn is not None:
            return
        self.render_path=render_path
        if self.render_path is not None:
            if os.path.exists(self.render_path):
                import shutil
                shutil.rmtree(self.render_path)
            os.mkdir(self.render_path)
        self.animation_fn=fn
        self.animation_op=op
        self.animation_dur=dur
        self.animation_frms=[]

class ErrVisualizer(GLVisualizer):
    def __init__(self,robot,data):
        GLVisualizer.__init__(self,None,robot,None)
        self.c,self.l2c,self.r2c,self.c2b,self.lPos,self.rPos,   \
        self.xsLeft,self.EEpsLeft,self.psLeft,  \
        self.xsRight,self.EEpsRight,self.psRight=data
        self.idL=self.idR=0
        
    def feedVertex(self,v):
        gl.glVertex3f(v[0],v[1],v[2])
        
    def drawEE(self,T,EEs,pos):
        gldraw.setcolor(0,0,0)
        gl.glBegin(gl.GL_LINES)
        for a,b in zip(EEs[:-1],EEs[1:]):
            a=a if not isinstance(a,tuple) else se3.apply(a,pos)
            b=b if not isinstance(b,tuple) else se3.apply(b,pos)
            self.feedVertex(se3.apply(T,a))
            self.feedVertex(se3.apply(T,b))
        gl.glEnd()

    def drawPs(self,T,ps):
        gldraw.setcolor(1,0,0)
        gl.glBegin(gl.GL_LINES)
        for a,b in zip(ps[:-1],ps[1:]):
            self.feedVertex(se3.apply(T,a))
            self.feedVertex(se3.apply(T,b))
        gl.glEnd()
    
    def drawRobot(self,x,T,links,drawLinks,pos):
        EEId=None
        c=self.robot.getConfig()
        for id,l in enumerate(links):
            c[l]=x[id]
            EEId=l
        self.robot.setConfig(c)
        
        for l in drawLinks:
            L=self.robot.link(l)
            T0=L.getTransform()
            T1=se3.mul(T,T0)
            L.setTransform(T1[0],T1[1])
            L.drawWorldGL()
            L.setTransform(T0[0],T0[1])
            
        L=self.robot.link(EEId+1)
        T0=L.getTransform()
        T1=se3.mul(T,T0)
        L.setTransform(T1[0],T1[1])
        gldraw.setcolor(0,1,0)
        gl.glBegin(gl.GL_LINES)
        self.feedVertex(se3.apply(T1,[0,0,0]))
        self.feedVertex(se3.apply(T1,pos))
        gl.glEnd()
        L.setTransform(T0[0],T0[1])
        
    def drawRobotOriginal(self):
        c=self.robot.getConfig()
        for id,l in enumerate(range(7,13)):
            c[l]=self.xsLeft[self.idL][id]
            EEIdL=l
        for id,l in enumerate(range(15,21)):
            c[l]=self.xsRight[self.idR][id]
            EEIdR=l
        self.robot.setConfig(c)
        self.robot.drawGL()
        
        L=self.robot.link(EEIdL+1)
        T0=L.getTransform()
        gldraw.setcolor(0,1,0)
        gl.glBegin(gl.GL_LINES)
        self.feedVertex(se3.apply(T0,[0,0,0]))
        self.feedVertex(se3.apply(T0,self.lPos))
        gl.glEnd()
        
        L=self.robot.link(EEIdR+1)
        T0=L.getTransform()
        gldraw.setcolor(0,1,0)
        gl.glBegin(gl.GL_LINES)
        self.feedVertex(se3.apply(T0,[0,0,0]))
        self.feedVertex(se3.apply(T0,self.rPos))
        gl.glEnd()
        
    def drawBase(self,T):
        L=self.robot.link(5)
        T0=L.getTransform()
        T1=se3.mul(T,T0)
        L.setTransform(T[0],T[1])
        L.drawWorldGL()
        L.setTransform(T0[0],T0[1])
        
    def drawTable(self,N=1,res=10):
        gldraw.setcolor(0,0,0)
        gl.glBegin(gl.GL_LINES)
        for i in range(res):
            d=float(i)*(2*N)/(res-1)-N
            self.feedVertex([-N,d,0])
            self.feedVertex([ N,d,0])
            self.feedVertex([d,-N,0])
            self.feedVertex([d, N,0])
        gl.glEnd()
        
    def drawCamera(self,T,sz=0.01):
        gldraw.setcolor(1,0,0)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glPushMatrix()
        gl.glTranslatef(T[1][0],T[1][1],T[1][2])
        gldraw.box((-sz,-sz,-sz),(sz,sz,sz))
        gl.glPopMatrix()
        
    def display(self):
        l=se3.mul(self.c,se3.inv(self.l2c))
        r=se3.mul(self.c,se3.inv(self.r2c))
        b=se3.mul(self.c,self.c2b)
        self.drawEE(l,self.EEpsLeft,self.lPos)
        self.drawEE(r,self.EEpsRight,self.rPos)
        self.drawPs(self.c,self.psLeft)
        self.drawPs(self.c,self.psRight)
        self.drawRobotOriginal()
        #self.drawRobot(self.xsLeft[self.idL],l,range(7,13),range(6,13),self.lPos)
        #self.drawRobot(self.xsRight[self.idR],r,range(15,21),range(14,21),self.rPos)
        #self.drawBase(b)
        self.drawTable()
        self.drawCamera(self.c)
        
    def idle(self):
        GLVisualizer.idle(self)
        self.idL=(self.idL+1)%len(self.xsLeft)
        self.idR=(self.idR+1)%len(self.xsRight)
        
def get_left(robot,x,lPos):
    c=robot.getConfig()
    c[7:13]=x[:6]
    robot.setConfig(c)
    Rt=robot.link(13).getTransform()
    if lPos is None:
        return Rt
    else: return se3.apply(Rt,lPos)
    
def get_right(robot,x,rPos):
    c=robot.getConfig()
    c[15:21]=x[:6]
    robot.setConfig(c)
    Rt=robot.link(21).getTransform()
    if rPos is None:
        return Rt
    else: return se3.apply(Rt,rPos)

def rotation_vec(R,res):
    return [-math.pi+math.pi*2*i/(res-1) for i in R]

def rotation(R,res):
    return so3.from_rotation_vector(rotation_vec(R,res))

def sos2(M,func,res):
    import gurobipy as gp
    vars=[0. for i in range(res)]
    for x in range(res):
        for y in range(res):
            for z in range(res):
                vars[z]+=func(x,y,z)
    varsTmp=[M.addVar(lb=0.,ub=1.) for i in range(res)]
    for i in range(res):
        M.addConstr(vars[i]==varsTmp[i])
    M.addSOS(gp.GRB.SOS_TYPE2,varsTmp)
    M.addConstr(sum(varsTmp)==1)
    return vars

def rotation_constraint(rot,M,res):
    rotC=[0. for i in range(9)]
    rotCNode=[[[rotation([x,y,z],res) for x in range(res)] for y in range(res)] for z in range(res)]
    rotCWeight=[[[M.addVar(ub=1) for x in range(res)] for y in range(res)] for z in range(res)]
    for x in range(res):
        for y in range(res):
            for z in range(res):
                rotC=vectorops.add(rotC,vectorops.mul(rotCNode[x][y][z],rotCWeight[x][y][z]))
    for i in range(9):
        M.addConstr(rotC[i]==rot[i])
    def funcx(x,y,z):
        return rotCWeight[x][y][z]
    xVar=sos2(M,funcx,res)
    def funcy(x,y,z):
        return rotCWeight[x][z][y]
    yVar=sos2(M,funcy,res)
    def funcz(x,y,z):
        return rotCWeight[z][x][y]
    zVar=sos2(M,funcz,res)
    return xVar,yVar,zVar,rotCWeight

def recover_rotation(rotCWeight,res):
    vec=[0. for i in range(3)]
    for x in range(res):
        for y in range(res):
            for z in range(res):
                vec=vectorops.add(vec,vectorops.mul(rotation_vec([x,y,z],res),rotCWeight[x][y][z]))
    return vec

def fetch_rotation(rot):
    xVar,yVar,zVar,rotCWeight=rot
    res=len(xVar)
    xVar=[x.getValue() for x in xVar]
    yVar=[y.getValue() for y in yVar]
    zVar=[z.getValue() for z in zVar]
    for x in range(res):
        for y in range(res):
            for z in range(res):
                rotCWeight[x][y][z]=rotCWeight[x][y][z].x
    return recover_rotation(rotCWeight,res)

def extract_se3(x,style):
    ret=[]
    for c in style:
        if c=='T':
            ret.append((so3.from_rotation_vector(x[:3]),x[3:6]))
            x=x[6:]
        elif c=='t':
            ret.append(x[:3])
            x=x[3:]
        else: raise RuntimeError("Unknown x type")
    if len(ret)==1:
        return ret[0]
    else: return tuple(ret)

def summarize_se3(x):
    return so3.rotation_vector(x[0])+x[1]

def extrinsic_calibration_solve_arm(EEsLeft,psLeft,EEsRight,psRight,callback=True,res=8):
    import gurobipy as gp
    #declare model, variable
    M=gp.Model("Calibration")
    M.setParam("LogToFile","")
    M.setParam("LogToConsole",1 if callback else 0)
    l2c=([M.addVar(lb=-1,ub=1) for i in range(9)],[M.addVar(lb=-gp.GRB.INFINITY) for i in range(3)])
    r2c=([M.addVar(lb=-1,ub=1) for i in range(9)],[M.addVar(lb=-gp.GRB.INFINITY) for i in range(3)])
    lPos=[M.addVar(lb=-gp.GRB.INFINITY) for i in range(3)]
    rPos=[M.addVar(lb=-gp.GRB.INFINITY) for i in range(3)]
    
    #rotation constraint
    lRot=rotation_constraint(l2c[0],M,res)
    rRot=rotation_constraint(r2c[0],M,res)
    
    #energy
    expr=0.
    for EE,p in zip(EEsLeft,psLeft):
        if isinstance(EE,tuple):
            EE=se3.apply(EE,lPos)
        EERef=se3.apply(l2c,p)
        EEDiff=vectorops.sub(EERef,EE)
        expr+=vectorops.dot(EEDiff,EEDiff)
    for EE,p in zip(EEsRight,psRight):
        if isinstance(EE,tuple):
            EE=se3.apply(EE,rPos)
        EERef=se3.apply(r2c,p)
        EEDiff=vectorops.sub(EERef,EE)
        expr+=vectorops.dot(EEDiff,EEDiff)
    M.setObjective(expr,gp.GRB.MINIMIZE)

    #solve
    M.optimize()
    if M.status==gp.GRB.OPTIMAL:
        x=fetch_rotation(lRot)+[c.x for c in l2c[1]]+fetch_rotation(rRot)+[c.x for c in r2c[1]]+[m.x for m in lPos]+[m.x for m in rPos]
        #optimize: local
        def func(x):
            expr=0.
            l2c,r2c,lPos,rPos=extract_se3(x,'TTtt')
            for EE,p in zip(EEsLeft,psLeft):
                if isinstance(EE,tuple):
                    EE=se3.apply(EE,lPos)
                EERef=se3.apply(l2c,p)
                EEDiff=vectorops.sub(EERef,EE)
                expr+=vectorops.dot(EEDiff,EEDiff)
            for EE,p in zip(EEsRight,psRight):
                if isinstance(EE,tuple):
                    EE=se3.apply(EE,rPos)
                EERef=se3.apply(r2c,p)
                EEDiff=vectorops.sub(EERef,EE)
                expr+=vectorops.dot(EEDiff,EEDiff)
            expr=math.sqrt(expr/(len(psLeft)+len(psRight)))
            if callback:
                print("Local solve: func(arm)=%f"%expr)
            return expr
        import scipy.optimize as sopt
        x=sopt.minimize(func,x,method='L-BFGS-B').x
        print("Local solve: func(arm)=%f"%func(x))
        l2c,r2c,lPos,rPos=extract_se3(x,'TTtt')
        return l2c,r2c,lPos,rPos,func(x)
    else: raise RuntimeError("Gurobi failed")

def extrinsic_calibration_solve_camera(psTable,callback=True,res=16):
    import gurobipy as gp
    #declare model, variable
    M=gp.Model("Calibration")
    M.setParam("LogToFile","")
    M.setParam("LogToConsole",1 if callback else 0)
    c=([M.addVar(lb=-1,ub=1) for i in range(9)],[M.addVar(lb=-gp.GRB.INFINITY) for i in range(3)])
    
    #rotation constraint
    rot=rotation_constraint(c[0],M,res)
    M.addConstr(c[1][2]>=0.1)
    
    #energy
    expr=0.
    for p in psTable:
        EERef=se3.apply(c,p)
        expr+=EERef[2]*EERef[2]
    M.setObjective(expr,gp.GRB.MINIMIZE)

    #solve
    M.optimize()
    if M.status==gp.GRB.OPTIMAL:
        x=fetch_rotation(rot)+[ci.x for ci in c[1]]
        #optimize: local
        def func(x):
            expr=0.
            c=extract_se3(x,'T')
            for p in psTable:
                EERef=se3.apply(c,p)
                expr+=EERef[2]*EERef[2]
            expr=math.sqrt(expr/(len(psTable)))
            if callback:
                print("Local solve: func(camera)=%f"%expr)
            return expr
        import scipy.optimize as sopt
        mins=[-math.pi*2]*3+[-10.,-10.,0.1]
        maxs=[ math.pi*2]*3+[ 10., 10.,10.]
        x=sopt.minimize(func,x,bounds=[i for i in zip(mins,maxs)],method='L-BFGS-B').x
        print("Local solve: func(camera)=%f"%func(x))
        c=extract_se3(x,'T')
        return c,func(x)
    else: raise RuntimeError("Gurobi failed")

def calibrate_URDF(c,l2c,r2c,robot):
    geom=robot.link(6).geometry().getTriangleMesh()
    vss=[[geom.vertices[j] for j in range(i*3,i*3+3)] for i in range(len(geom.vertices)//3)]
    
    c2l=se3.inv(l2c)
    c2r=se3.inv(r2c)
    A=np.zeros((3,len(vss*2)))
    B=np.zeros((3,len(vss*2)))
    for id,v in enumerate(vss):
        A[:,id*2+0]=v
        B[:,id*2+0]=se3.apply(c2l,v)
        A[:,id*2+1]=v
        B[:,id*2+1]=se3.apply(c2r,v)
    
    Am = A-np.expand_dims(np.mean(A,1),1)
    Bm = B-np.expand_dims(np.mean(B,1),1)
    U, S, Vt = np.linalg.svd(np.matmul(Bm,Am.T))
    SDiag = np.identity(3,dtype=np.float64)
    SDiag[2,2] = np.linalg.det(np.matmul(U,Vt))
    R = np.matmul(U,np.matmul(SDiag,Vt))
    #print(np.linalg.det(R))
    pos = np.mean(B,1) - np.matmul(R,np.mean(A,1))
    err = np.matmul(R,A)+np.expand_dims(pos,1)-B
    return ([R[0,0],R[1,0],R[2,0],R[0,1],R[1,1],R[2,1],R[0,2],R[1,2],R[2,2]],[pos[0],pos[1],pos[2]])

def calibrate_desired_base_mesh(robot,c,l2c,r2c,c2b,vss0):
    geom=robot.link(5).geometry().getTriangleMesh()
    T=se3.mul(c,c2b)
    vss=[se3.apply(T,[geom.vertices[j] for j in range(i*3,i*3+3)]) for i in range(len(geom.vertices)//3)]
    if len(vss)!=len(vss0):
        raise RuntimeError("Desired base mesh does not have same number of vertices (%d) as that of current base mesh (%d)"%(len(vss),len(vss0)))
    
    id=0
    A=np.zeros((2,len(vss)))
    B=np.zeros((2,len(vss)))
    for v,v0 in zip(vss,vss0):
        A[:,id]=v[:2]
        B[:,id]=v0[:2]
        id+=1
    
    Am = A-np.expand_dims(np.mean(A,1),1)
    Bm = B-np.expand_dims(np.mean(B,1),1)
    U, S, Vt = np.linalg.svd(np.matmul(Bm,Am.T))
    SDiag = np.identity(2,dtype=np.float64)
    SDiag[1,1] = np.linalg.det(np.matmul(U,Vt))
    R = np.matmul(U,np.matmul(SDiag,Vt))
    #print(np.linalg.det(R))
    pos = np.mean(B,1) - np.matmul(R,np.mean(A,1))
    err = np.matmul(R,A)+np.expand_dims(pos,1)-B
    cT=([R[0,0],R[1,0],0,R[0,1],R[1,1],0,0,0,1],[pos[0],pos[1],0])
    return se3.mul(cT,c)

def T_2_xyz_rpy(T):
    import trimesh
    xyz=T[:3,3].tolist()
    rpy=list(trimesh.transformations.euler_from_matrix(T[:3,:3]))
    return xyz,rpy

def adjust_URDF_file(robot,c,l2c,r2c,c2b,ROBOT_PATH,ROBOT_PATH_OUT=None):
    import lxml.etree as ET
    assert ROBOT_PATH.endswith(".urdf")
    c2l=se3.mul(se3.inv(l2c),robot.link(6).getTransform())
    c2r=se3.mul(se3.inv(r2c),robot.link(14).getTransform())
    lT=se3.mul(c,c2l)
    rT=se3.mul(c,c2r)
    bT=se3.mul(c,c2b)
    
    str=open(ROBOT_PATH,'r').read().encode()
    root=ET.fromstring(str)
    leftJoint=[l for l in root.findall("joint") if l.get("name")=="base_left_base"][0].find("origin")
    T=np.identity(4,np.float64)
    T[:3,0]=lT[0][0:3]
    T[:3,1]=lT[0][3:6]
    T[:3,2]=lT[0][6:9]
    T[:3,3]=lT[1]
    xyz,rpy=T_2_xyz_rpy(T)
    leftJoint.set("xyz","%f %f %f"%(xyz[0],xyz[1],xyz[2]))
    leftJoint.set("rpy","%f %f %f"%(rpy[0],rpy[1],rpy[2]))
    
    rightJoint=[l for l in root.findall("joint") if l.get("name")=="base_right_base"][0].find("origin")
    T=np.identity(4,np.float64)
    T[:3,0]=rT[0][0:3]
    T[:3,1]=rT[0][3:6]
    T[:3,2]=rT[0][6:9]
    T[:3,3]=rT[1]
    xyz,rpy=T_2_xyz_rpy(T)
    rightJoint.set("xyz","%f %f %f"%(xyz[0],xyz[1],xyz[2]))
    rightJoint.set("rpy","%f %f %f"%(rpy[0],rpy[1],rpy[2]))
    
    import trimesh
    T=np.identity(4,np.float64)
    T[:3,0]=bT[0][0:3]
    T[:3,1]=bT[0][3:6]
    T[:3,2]=bT[0][6:9]
    T[:3,3]=bT[1]
    e=[l for l in root.findall("link") if l.get("name")=="base_link"][0]
    path0=e.find("visual").find("geometry").find("mesh").get("filename")
    mesh=trimesh.exchange.load.load(os.path.dirname(ROBOT_PATH)+"/"+path0)
    mesh.apply_transform(T).export(os.path.dirname(ROBOT_PATH)+"/"+path0[:-4]+"_Calibrated.STL")
    
    assert path0.endswith(".STL")
    visualMesh=e.find("visual").find("geometry").find("mesh")
    visualMesh.set("filename",path0[:-4]+"_Calibrated.STL")
    collisionMesh=e.find("collision").find("geometry").find("mesh")
    collisionMesh.set("filename",path0[:-4]+"_Calibrated.STL")
    
    pathOut=ROBOT_PATH[:-5]+"_Calibrated.urdf" if ROBOT_PATH_OUT is None else ROBOT_PATH_OUT
    open(pathOut,'w').write(ET.tostring(root,pretty_print=True).decode())
    return pathOut

def detect_aruco(frame,markerSz,markerSzTable,dict,dictTable,lMarker,rMarker,mtx,dist,NTimes=1):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if mtx is None or dist is None:
        if frame.shape==(1920,1080):
            fx=1055.28
            fy=1054.69
            cx=982.61
            cy=555.138
            k1=-0.0426844
            k2=0.0117943
            k3=-0.00548354
            p1=0.000242741
            p2=-0.000475926
        else:
            fx=1055.28
            fy=1054.69
            cx=1126.61
            cy=636.138
            k1=-0.0426844
            k2=0.0117943
            k3=-0.00548354
            p1=0.000242741
            p2=-0.000475926
        mtx = np.array([[fx,0,cx], [0,fy,cy], [0,0,1]])
        dist = np.array([k1,k2,p1,p2,k3])
    parameters = aruco.DetectorParameters_create()
    aruco_dict = aruco.Dictionary_get(dict)
    aruco_dict_table = aruco.Dictionary_get(dictTable)
    
    #lr
    pl,pr=None,None
    nl,nr=0,0
    for time in range(NTimes):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, \
                                                              cameraMatrix=mtx, distCoeff=dist)        
        if len(corners)>0:
            for c,id in zip(corners,ids):
                if id==lMarker:
                    rvec,tvec,trash=aruco.estimatePoseSingleMarkers(c,markerSz,mtx,dist)
                    pl=tvec[0,0,:].tolist() if pl is None else vectorops.add(pl,tvec[0,0,:].tolist())
                    nl+=1
                elif id==rMarker:
                    rvec,tvec,trash=aruco.estimatePoseSingleMarkers(c,markerSz,mtx,dist)
                    pr=tvec[0,0,:].tolist() if pr is None else vectorops.add(pr,tvec[0,0,:].tolist())
                    nr+=1
    if pl is not None:
        pl=vectorops.div(pl,nl)
    if pr is not None:
        pr=vectorops.div(pr,nr)
    
    #t
    pt=[]
    nt=[]
    for time in range(NTimes):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict_table, parameters=parameters, \
                                                              cameraMatrix=mtx, distCoeff=dist)     
        if len(corners)>0:
            indices=[]
            for c,id in zip(corners,ids):
                rvec,tvec,trash=aruco.estimatePoseSingleMarkers(c,markerSzTable,mtx,dist)
                if time==0:
                    pt.append(tvec[0,0,:].tolist())
                    nt.append(1)
                else:
                    dists=[vectorops.distance(tvec[0,0,:].tolist(),vectorops.div(p,n)) for p,n in zip(pt,nt)]
                    index=dists.index(min(dists))
                    assert index not in indices
                    indices.append(index)
                    
                    pt[index]=vectorops.add(pt[index],tvec[0,0,:].tolist())
                    nt[index]+=1
    if len(pt)>0:
        pt=[vectorops.div(p,n) for p,n in zip(pt,nt)]
        
    return pl,pr,pt
    
def extrinsic_calibration_read(folder,dict=aruco.DICT_ARUCO_ORIGINAL,dictTable=aruco.DICT_7X7_50,
                     lMarker=2,rMarker=1,lPos=None,rPos=None,mtx=None,dist=None,
                     markerSz=0.0378,markerSzTable=0.052,
                     ROBOT_PATH=".",ROBOT_PATH_OUT=None,
                     desired_base_mesh=None,    #provide the vertices of base mesh to fit URDF, skip this if None provided
                     vis=True):
    #collect data
    import pickle
    world=WorldModel()
    robot=world.loadRobot(ROBOT_PATH)
    if os.path.exists(folder+'/detected_aruco.dat'):
        xsLeft,EEpsLeft,psLeft,xsRight,EEpsRight,psRight,psTable=pickle.load(open(folder+'/detected_aruco.dat','rb'))
    else:
        EEs=[]
        EEStamps=[]
        for l in open(folder+'/robot_state.txt','r').readlines():
            vss=[float(v) for v in l.split(' ') if v!='\n']
            EEStamps.append(float(vss[0]))
            EEs.append(vss[1:13])
            
        xsLeft,xsRight=[],[]
        EEpsLeft,EEpsRight=[],[]
        psLeft,psRight,psTable=[],[],[]
        for il,l in enumerate(open(folder+'/camera_stamps.txt','r').readlines()):
            frame=cv2.imread(folder+"/color_overhead-%05d.jpg"%il)
            pl,pr,pt=detect_aruco(frame,markerSz,markerSzTable,dict,dictTable,lMarker,rMarker,mtx,dist)
            if pl is not None or pr is not None:
                t=float(l)
                for j in range(len(EEStamps)-1):
                    if EEStamps[j]<=t and EEStamps[j+1]>=t:
                        alpha=(t-EEStamps[j])/(EEStamps[j+1]-EEStamps[j])
                        x=vectorops.add(vectorops.mul(EEs[j],1-alpha),vectorops.mul(EEs[j+1],alpha))
                        break
            info=""
            if pl is not None:
                EEpsLeft.append(get_left(robot,x[:6],lPos))
                xsLeft.append(x[:6])
                psLeft.append(pl)
                info+="[left]"
            if pr is not None:
                EEpsRight.append(get_right(robot,x[6:],rPos))
                xsRight.append(x[6:])
                psRight.append(pr)
                info+="[right]"
            if pt is not None:
                if len(pt)>len(psTable):
                    psTable=pt
                info+="[table:%d]"%len(psTable)
            print('Found %s marker in frame: %d'%(info,il))
        pickle.dump((xsLeft,EEpsLeft,psLeft,xsRight,EEpsRight,psRight,psTable),open(folder+'/detected_aruco.dat','wb'))
    
    #solve optimization: arm
    if os.path.exists(folder+'/calibration_arm.dat'):
        l2c,r2c,lPos,rPos=pickle.load(open(folder+'/calibration_arm.dat','rb'))
    else:
        l2c,r2c,lPos,rPos,_=extrinsic_calibration_solve_arm(EEpsLeft,psLeft,EEpsRight,psRight)
        pickle.dump((l2c,r2c,lPos,rPos),open(folder+'/calibration_arm.dat','wb'))

    #solve optimization: camera
    if os.path.exists(folder+'/calibration_camera.dat'):
        c,l2c,r2c,lPos,rPos=pickle.load(open(folder+'/calibration_camera.dat','rb'))
    else:
        c,_=extrinsic_calibration_solve_camera(psTable)
        pickle.dump((c,l2c,r2c,lPos,rPos),open(folder+'/calibration_camera.dat','wb'))
    
    #calibrate URDF
    c2b=calibrate_URDF(c,l2c,r2c,robot)
    if desired_base_mesh is not None:
        c=calibrate_desired_base_mesh(robot,c,l2c,r2c,c2b,desired_base_mesh)
    ROBOT_PATH_OUT=adjust_URDF_file(robot,c,l2c,r2c,c2b,ROBOT_PATH,ROBOT_PATH_OUT)
    
    #visualize
    if vis:
        world=WorldModel()
        robot=world.loadRobot(ROBOT_PATH_OUT)
        ErrVisualizer(robot,(c,l2c,r2c,c2b,lPos,rPos,xsLeft,EEpsLeft,psLeft,xsRight,EEpsRight,psRight)).run()
    return c if desired_base_mesh is not None else c2b
         
def read_chessboards(images,dict,boardDef):
    """
    Charuco base pose estimation.
    """
    aruco_dict = aruco.Dictionary_get(dict)
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0

    for idm,im in enumerate(images):
        print("=> Processing image %d"%idm)
        if isinstance(im,str):
            frame = cv2.imread(im)
        else: frame=im
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        res = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(res[0])>0:
            res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,boardDef)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>6 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator+=1

    imsize = gray.shape
    print("FINISHED!")
    return allCorners,allIds,imsize
    
def calibrate_camera_cv2(allCorners,allIds,imsize,boardDef):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")
    cameraMatrixInit = np.array([[1055.28,     0.,1126.61],
                                 [     0.,1054.69,636.138],
                                 [     0.,     0.,     1.]])
    distCoeffsInit = np.array([-0.0426844,0.0117943,0.000242741,-0.000475926,-0.00548354])
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=boardDef,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))
    print("FINISHED!")
    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors
               
def intrinsic_calibration_read(folder,alpha=0.8,dict=aruco.DICT_ARUCO_ORIGINAL):
    import pickle
    if os.path.exists(folder+'/intrinsic_parameters.dat'):
        mtx,dist=pickle.load(open(folder+'/intrinsic_parameters.dat','rb'))
    else:
        minDiff=None
        maxDiff=None
        lastFrame=None
        for il,l in enumerate(open(folder+'/camera_stamps.txt','r').readlines()):
            frame=cv2.imread(folder+"/color_overhead-%05d.jpg"%il)
            if lastFrame is not None:
                diff=np.linalg.norm(frame-lastFrame)
                minDiff=diff if minDiff is None else min(minDiff,diff)
                maxDiff=diff if maxDiff is None else max(maxDiff,diff)
            lastFrame=frame
            
        images=[]
        for il,l in enumerate(open(folder+'/camera_stamps.txt','r').readlines()):
            frame=cv2.imread(folder+"/color_overhead-%05d.jpg"%il)
            if lastFrame is not None:
                diff=np.linalg.norm(frame-lastFrame)
                if diff<minDiff*alpha+maxDiff*(1-alpha):
                    images.append(frame)
            lastFrame=frame
            
        import matplotlib.pyplot as plt
        print("Calibrating using %d images!"%len(images))
        if len(images)==0:
            raise RuntimeError("Cannot find images without motion blur")
        for img in images:
            plt.imshow(img)
            plt.show()
        
        aruco_dict = aruco.Dictionary_get(dict)
        boardDef = aruco.CharucoBoard_create(5,5,0.05,0.04,aruco_dict)
        allCorners, allIds, imsize = read_chessboards(images,dict,boardDef)
        _,mtx,dist,_,_=calibrate_camera_cv2(allCorners,allIds,imsize,boardDef)
        pickle.dump((mtx,dist),open(folder+'/intrinsic_parameters.dat','wb'))
    return mtx,dist
              
if __name__=='__main__':
    from scene import make_scene_TRINA
    world,table,robot,robot_path=make_scene_TRINA()
    geom=robot.link(5).geometry().getTriangleMesh()
    T=robot.link(5).getTransform()
    vss0=[se3.apply(T,[geom.vertices[j] for j in range(i*3,i*3+3)]) for i in range(len(geom.vertices)//3)]
    #vss0=None
    
    extrinsic=extrinsic_calibration_read( folder='../calibration_1',lPos=None,rPos=None,
                                          ROBOT_PATH="./data/TRINA/robots/Anthrax_lowpoly_Adjusted.urdf",
                                          #ROBOT_PATH_OUT="../Calibrated.urdf",
                                          desired_base_mesh=vss0,
                                          vis=False)
    print(extrinsic)
    