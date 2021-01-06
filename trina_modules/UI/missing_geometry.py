from klampt import WorldModel,Geometry3D
from klampt import vis

#you will need to change this to the absolute or relative path to Klampt-examples
KLAMPT_EXAMPLES = '.'

shelf_dims = (0.4,0.4,0.3)
shelf_offset_x=0.8
shelf_offset_y = 0.1
shelf_height = 0.65

def make_shelf(world,width,depth,height,wall_thickness=0.005):
    """Makes a new axis-aligned "shelf" centered at the origin with
    dimensions width x depth x height. Walls have thickness wall_thickness.
    If mass=inf, then the box is a Terrain, otherwise it's a RigidObject
    with automatically determined inertia.
    """
    left = Geometry3D()
    right = Geometry3D()
    back = Geometry3D()
    bottom = Geometry3D()
    top = Geometry3D()
    left.loadFile(KLAMPT_EXAMPLES+"/cube.off")
    right.loadFile(KLAMPT_EXAMPLES+"/cube.off")
    back.loadFile(KLAMPT_EXAMPLES+"/cube.off")
    bottom.loadFile(KLAMPT_EXAMPLES+"/cube.off")
    top.loadFile(KLAMPT_EXAMPLES+"/cube.off")
    left.transform([wall_thickness,0,0,0,depth,0,0,0,height],[-width*0.5,-depth*0.5,0])
    right.transform([wall_thickness,0,0,0,depth,0,0,0,height],[width*0.5,-depth*0.5,0])
    back.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,depth*0.5,0])
    bottom.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,0])
    top.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,height-wall_thickness])
    shelfgeom = Geometry3D()
    shelfgeom.setGroup()
    for i,elem in enumerate([left,right,back,bottom,top]):
        g = Geometry3D(elem)
        shelfgeom.setElement(i,g)
    shelf = world.makeTerrain("shelf")
    shelf.appearance().setDraw(0,False)
    shelf.geometry().set(shelfgeom)
    shelf.appearance().setColor(0.2,0.6,0.3,1.0)
    shelf.appearance().setShininess(0)
    
    return shelf

w = WorldModel()

shelf = make_shelf(w,*shelf_dims)
shelf.geometry().translate((shelf_offset_x,shelf_offset_y,shelf_height))

vis.debug(shelf)
