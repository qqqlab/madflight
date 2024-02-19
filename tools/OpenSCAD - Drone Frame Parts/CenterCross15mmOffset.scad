// Center cross for a drone frame

//print with 100% infill

//=======================================================================
// Config
//=======================================================================
//beam diameter
dia1=15.8;

//wall thickness main cross beam
wall=2; 

//socket depth for both axis
socket1=40;

//plate thickness
plate_h = 6;

//strap holder
strap_l=0; //set to 0 to not use straps
strap_w=3;
strap_edge = 4;

//=======================================================================

$fn=60; //number of fragments

tol=0.01;


plate_w = 53;

difference() {

  union() {
    rotate([0,0,45]) Tube_add(2*socket1,dia1/2,2);
    translate([0,0,dia1]) rotate([0,0,-45]) Tube_add(2*socket1,dia1/2,wall);
    
    translate([0,0,dia1]) rotate([0,0,45]) Tube_add(2*socket1,dia1/2,2);
    rotate([0,0,-45]) Tube_add(2*socket1,dia1/2,wall);
    
    //plate
    translate([-plate_w/2,-plate_w/2,0]) cube([plate_w,plate_w,plate_h]);
  }
  rotate([0,0,45]) Tube_drill(2*socket1,dia1/2,2);
  translate([0,0,dia1]) rotate([0,0,-45]) Tube_drill(2*socket1,dia1/2,wall);
  
  //strap holes
  if(strap_l) {
    strap_offset = plate_w/2-strap_edge-strap_w/2;
    Strap_drill(strap_offset);
    Strap_drill(-strap_offset);
  }
  
  //motor M4xdia40 holes
  translate([20,0,0]) cylinder(h=1000,r=2.25,center=true);
  translate([0,20,0]) cylinder(h=1000,r=2.25,center=true);
  translate([-20,0,0]) cylinder(h=1000,r=2.25,center=true);
  translate([0,-20,0]) cylinder(h=1000,r=2.25,center=true);
  
  //keep bottom half only
  //translate([-150,-150,dia1/2+wall]) cube([300,300,50]);
}







//Tube along x-axis in positive z
module Tube_add(h, r, wall) {
  translate([-h/2,0,r+wall]) rotate([0,90,0]) 
    //cylinder(h=h, r=r+wall);
    translate([-r-wall,-r-wall,0]) cube([2*(r+wall),2*(r+wall),h]);
}
module Tube_drill(h, r, wall) {
  translate([-500,0,r+wall]) rotate([0,90,0]) 
    cylinder(h=1000, r=r);
}
  
module Strap_drill(offset) {
  translate([0,offset,0]) union() {
    cube([strap_l,strap_w,1000],center=true);
    translate([-strap_l/2,0,0]) cylinder(h=1000,r=strap_w/2,center=true);
    translate([+strap_l/2,0,0]) cylinder(h=1000,r=strap_w/2,center=true);
    //translate([0,0,500+strap_edge]) cube([strap_l+2*strap_edge,strap_w+2*strap_edge+tol,1000],center=true);
  }
}
