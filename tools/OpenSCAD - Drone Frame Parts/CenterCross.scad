// Center cross for a drone frame

//print with 100% infill

//=======================================================================
// Config
//=======================================================================
//beam diameter
dia1=6.5;

//wall thickness main cross beam
wall=1.5; //1mm is too thin: cross breaks before beam

//socket depth for both axis
//as a rule, make socket 2*diameter deep
socket1=dia1; //socket has a through beam, so only half the socket needed 
socket2=2.5*dia1; //.5 diameter is blocked by other beam, so 2.5 diameter in total

//z-offset between the two axis
//z_offset=6.5;
z_offset=0;

//use hull?
use_hull=0;

//flat top/bottom?
use_flat_top=0;
use_flat_bottom=0;

//strap holder
strap_l=20; //set to 0 to not use straps
strap_w=2; 
strap_h=2;
strap_offset = 3; //offset from xy axis
strap_reinforce = 1; //0=no, 1=yes

//=======================================================================

$fn=60; //number of fragments

difference() {
  union() {
    if(strap_h>0) 
      //translate([0,0,-strap_h+wall]) 
        Straps(w=strap_w,h=strap_h,l=strap_l,offset=strap_offset);
    Cross();
  }
  CrossDrill();
}


//------------------------------------------------------------------------------------------------
module Cross() {
  od = dia1+2*wall; //outside diameter

  translate([0,0,od/2])
  HullOrUnion() {
    //outline
    translate([0,socket1,-z_offset/2]) rotate([90,0,0]) cylinder(h=socket1*2,d=od);
    translate([socket2,0,z_offset/2]) rotate([0,-90,0]) cylinder(h=socket2*2,d=od);
  }
  if(use_flat_top) {
    translate([0,0,3*od/4]) cube([2*socket2,od,od/2],center=true);
    translate([0,0,3*od/4]) cube([od,2*socket1,od/2],center=true);
  }
  if(use_flat_bottom) {
    translate([0,0,1*od/4]) cube([2*socket2,od,od/2],center=true);
    translate([0,0,1*od/4]) cube([od,2*socket1,od/2],center=true);
  }  
}

module CrossDrill() {
  od = dia1+2*wall; //outside diameter

  translate([0,0,od/2])
  union() {
    //drill holes
    translate([0,0,-z_offset/2]) rotate([90,0,0]) cylinder(h=999,d=dia1,center=true);
    translate([0,0,z_offset/2]) rotate([0,-90,0]) cylinder(h=999,d=dia1,center=true);
  }
}

module HullOrUnion() {
  if(use_hull) {
    hull() children();
  }
  else
    union() children();
}

//------------------------------------------------------------------------------------------------
module Straps(w,h,l,offset) {
  d = offset;
  union() {
    //straps
    translate([d,d]) Strap(w,h,l);
    translate([-d,-d]) rotate([0,0,180]) Strap(w,h,l);
    //offset filler
    if(strap_reinforce) {
      //heavy
      translate([0,0,h/2]) cube([2*l-2.2,2*d+w,h],center=true);
      translate([0,0,h/2]) cube([2*d+w,2*l-2.2,h],center=true);
    }else{
      //light
      translate([0,0,h/2]) cube([2*socket2,2*d,h],center=true);
      translate([0,0,h/2]) cube([2*d,2*socket1,h],center=true);
    }

  }
}

module Strap(w,h,l) {
  r1=1;
  r2=r1+w;

  off1 = l/2;
  d = w*sqrt(2)/2;
  off2 = off1 + 2*d;

  difference() {
    hull() {
      translate([d,d]) cylinder(r=r2,h=h);
      translate([off2,d]) cylinder(r=r2,h=h);
      translate([d,off2]) cylinder(r=r2,h=h);
    }
    translate([r1+w/2,r1+w/2])
    hull() {
      translate([0,0]) cylinder(r=r1,h=999,center=true);
      translate([off1,0]) cylinder(r=r1,h=999,center=true);
      translate([0,off1]) cylinder(r=r1,h=999,center=true);
    }
  }
}