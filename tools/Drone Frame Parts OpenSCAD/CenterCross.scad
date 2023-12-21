// Center cross for a drone frame

//=======================================================================
// Config
//=======================================================================
//beam diameter
dia1=6.5;

//wall thickness main cross beam
wall=1.5;

//socket depth for both axis
socket1=dia1;
socket2=2*socket1+dia1/2;

//z-offset between the two axis
//z_offset=6.5;
z_offset=0;

//stiffening plate
plate_thickness=2;

//use hull?
use_hull=1;
//=======================================================================

$fn=60;  //number of facets
out1 = dia1+2*wall; //outside diameter

difference() {
  HullOrUnion() {
    //outline
    //rotate([0,0,45]) cube([(socket+dia1/2-1)*sqrt(2),(socket+dia1/2-1)*sqrt(2),plate_thickness],center=true);
    translate([0,socket1,-z_offset/2]) rotate([90,0,0]) cylinder(h=socket1*2,d=out1);
    translate([socket2,0,z_offset/2]) rotate([0,-90,0]) cylinder(h=socket2*2,d=out1);
  }

  //drill holes
  translate([0,socket1*2,-z_offset/2]) rotate([90,0,0]) cylinder(h=socket1*4,d=dia1);
  translate([socket2*2,0,z_offset/2]) rotate([0,-90,0]) cylinder(h=socket2*4,d=dia1);
}

module HullOrUnion() {
  if(use_hull) {
    hull() children();
  }
  else
    union() children();
}