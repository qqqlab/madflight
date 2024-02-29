// Beam Joint

//=======================================================================
// Config
//=======================================================================
//hole inner diameter
id=6.5;

//wall thickness main cross beam
wall=1.5; //1mm is too thin: cross breaks before beam

//socket depth for axis direction, set to 0 for no socket
//as a rule, make socket 2*id deep

/* 
// T
socketXp=0.5*id; //positive X axis
socketXn=0.5*id; //negative X axis
socketYp=2*id; //positive Y axis
socketYn=0*id; //negative Y axis
socketZp=0*id; //positive Z axis
socketZn=0*id; //negative Z axis
*/

// Cross
socketXp=2*id; //positive X axis
socketXn=2*id; //negative X axis
socketYp=2*id; //positive Y axis
socketYn=2*id; //negative Y axis
socketZp=0*id; //positive Z axis
socketZn=0*id; //negative Z axis

//use hull?
use_hull=0;

//flat top/bottom?
use_flat_bottom=0;
use_flat_top=0;

//=======================================================================

$fn=60; //number of fragments
tol = 0.001; //tolerance
od = id+2*wall; //outside diameter
or = od/2; //outside radius
ir = id/2; //inner radius

X = [1,0,0];
Y = [0,1,0];
Z = [0,0,1];

difference() {
  Cross();
  CrossDrill();
}

//------------------------------------------------------------------------------------------------
module Cross() {
  HullOrUnion() {
    sphere(d=od);
    if(socketXp>0) rotate([0,90,0]) cylinder(h=socketXp+ir,d=od);
    if(socketXn>0) rotate([0,-90,0]) cylinder(h=socketXn+ir,d=od);
    if(socketYp>0) rotate([-90,0,0]) cylinder(h=socketYp+ir,d=od);
    if(socketYn>0) rotate([90,0,0]) cylinder(h=socketYn+ir,d=od);
    if(socketZp>0) cylinder(h=socketZp+or,d=od);
    if(socketZn>0) rotate([180,0,0]) cylinder(h=socketZn+or,d=od);
  }
  if(use_flat_top) {
    zoff = or/2;
    translate([+(socketXp/2),0,zoff]) cube([socketXp+id,od,or],center=true);
    translate([-(socketXn/2),0,zoff]) cube([socketXn+id,od,or],center=true);
    translate([0,+(socketYp/2),zoff]) cube([od,socketYp+id,or],center=true);
    translate([0,-(socketYn/2),zoff]) cube([od,socketYn+id,or],center=true);
  }
  if(use_flat_bottom) {
    zoff = -or/2;
    translate([+(socketXp/2),0,zoff]) cube([socketXp+id,od,or],center=true);
    translate([-(socketXn/2),0,zoff]) cube([socketXn+id,od,or],center=true);
    translate([0,+(socketYp/2),zoff]) cube([od,socketYp+id,or],center=true);
    translate([0,-(socketYn/2),zoff]) cube([od,socketYn+id,or],center=true);
  }
}

module CrossDrill() {
  union() {
    sphere(d=id);
    if(socketXp>0) translate(-tol*X) rotate([0,90,0]) cylinder(h=999,d=id);
    if(socketXn>0) translate(+tol*X) rotate([0,-90,0]) cylinder(h=999,d=id);
    if(socketYp>0) translate(-tol*Y) rotate([-90,0,0]) cylinder(h=999,d=id);
    if(socketYn>0) translate(+tol*Y) rotate([90,0,0]) cylinder(h=999,d=id);
    if(socketZp>0) translate(-tol*Z) cylinder(h=999,d=id);
    if(socketZn>0) translate(+tol*Z) rotate([180,0,0]) cylinder(h=999,d=id);
  }
}

module HullOrUnion() {
  if(use_hull) {
    hull() children();
  }
  else
    union() children();
}

