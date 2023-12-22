// Center mounting plate for a drone frame, print with low density fill

//=======================================================================
// Config
//=======================================================================
//beam diameter
dia1=6.5;

//wall thickness main cross beam
wall=1.5;

//mounting plate
plate_w = 25;
plate_l = 80;
plate_h = .6;

//strap
strap_l = 18;
strap_w = 2;


edge_w = 2;
edge_h = 2;

//=======================================================================

$fn=60;  //number of facets
out1 = dia1+2*wall; //outside diameter


strap_y = plate_w/2-strap_w/2-edge_h;
  
difference() {
  union() {
    translate([0,0,-out1/2+plate_h/2]) cube([plate_l,plate_w,edge_h],center=true);
    
    //strengthen edges
    //translate([0,+(strap_y+strap_w/2+strap_edge/2),-out1/2+strap_h/2]) cube([plate_l,strap_edge,strap_h],center=true);
    //translate([0,-(strap_y+strap_w/2+strap_edge/2),-out1/2+strap_h/2]) cube([plate_l,strap_edge,strap_h],center=true);
  }
  cube([plate_l-edge_w*2,plate_w-2*edge_w,out1-2*plate_h],center=true);

  //drill holes
  //rotate([90,0,45]) cylinder(h=999,d=dia1,center=true);
  //rotate([90,0,-45]) cylinder(h=999,d=dia1,center=true);
  
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
