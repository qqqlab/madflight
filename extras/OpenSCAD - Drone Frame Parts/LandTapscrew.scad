dia = 15; //beam diameter
dia_gap = 0.5; //gap between two halves
screw = 3.8; //screw hole size
screw_to_beam = 0.5; //distance between beam and screw
wall = 1.5; //wall thickness around screw hole

hex_dia = 7.2; //hex bold counter sunk diameter M3=6.4 (set to 0 to ignore)
hex_depth = 1.5; //hex bold counter sunk depth M3=2.0

r = dia/2;
r_screw = screw/2;

land_dia = 6.5;

$fn=60;

screw2 = 2;

w = dia + 2 * (screw_to_beam + screw + wall); //width
d = screw + 2*wall; //depth
h = r + wall; //height


land_r = land_dia/2;
land_or = land_r+wall;


//rotate([-90,0,0])
difference() {
  union() {
    translate([0,0,(h-dia_gap/2)/2]) cube([w,d,h-dia_gap/2],center=true);

    //land
    translate([0,0,-land_or+2*wall]) rotate([0,90,0])cylinder(h=w,r=land_or,center=true);
    translate([0,0,-land_or+2*wall-land_or+2]) cube([w,d,4],center=true);
  }
  //beam
  translate([0,0,h]) rotate([90,0,0]) cylinder(h=100,r=r,center=true);

  //screws
  screw_offset = r + r_screw + screw_to_beam;
  translate([+screw_offset,0,+50]) cylinder(h=100,r=screw2/2,center=true);
  translate([-screw_offset,0,+50]) cylinder(h=100,r=screw2/2,center=true);
  
  
  //land
  translate([wall,0,-land_or+2*wall]) rotate([0,90,0])cylinder(h=w,r=land_r,center=true);
}

