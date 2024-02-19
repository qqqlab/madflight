// Center mounting plate for a drone frame

/* 3D Printer Settings:
wall line count: 5
top/bottom layers: 0
infill density: 30%
infill pattern: grid (ideal for tiewraps)
*/

//=======================================================================
// Config
//=======================================================================
//beam diameter
dia1=6.5;

//wall thickness main cross beam
wall=1.5;

//mounting plate
plate_w = 40;
plate_l = 26;
plate_h = .6;

//strap
strap_l = 18;
strap_w = 2;

//edge
edge_w = 2;
edge_h = 3;

mink_r = 4;
//=======================================================================

$fn=60;  //number of fragments
out1 = dia1+2*wall; //outside diameter
strap_y = plate_w/2-strap_w/2-edge_h; //strap center y pos
  
difference() {
  minkowski() {
    //plate with edge thickness
    translate([0,0,-out1/2+plate_h/2]) cube([plate_l-2*mink_r,plate_w-2*mink_r,edge_h],center=true);
    cylinder(r=mink_r,h=0.01);    
  }
  //remove excess plate thickness, except for edge
  //cube([plate_l-edge_w*2,plate_w-2*edge_w,out1-2*plate_h],center=true);

  //drill strap holes
  translate([0,+strap_y,0]) StrapHole();
  translate([0,-strap_y,0]) StrapHole();
}

module StrapHole() {
  union() {
    cube([strap_l,strap_w,999],center=true);
    translate([+strap_l/2,0,0]) cylinder(h=999,d=strap_w,center=true);
    translate([-strap_l/2,0,0]) cylinder(h=999,d=strap_w,center=true);    
  }
}
