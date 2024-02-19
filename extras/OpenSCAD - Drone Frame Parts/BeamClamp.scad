dia = 15; //beam diameter
dia_gap = 0.5; //gap between two halves
screw = 3.8; //screw hole size
screw_to_beam = 0.5; //distance between beam and screw
wall = 2; //wall thickness around screw hole

hex_dia = 7; //hex bold counter sunk diameter M3=6.4 (set to 0 to ignore)
hex_depth = 1.5; //hex bold counter sunk depth M3=2.0

r = dia/2;
r_screw = screw/2;

$fn=60;

w = dia + 2 * (screw_to_beam + screw + wall); //width
d = screw + 2*wall; //depth
h = r + wall; //height

difference() {
  translate([0,0,(h-dia_gap)/2]) cube([w,d,h-dia_gap/2],center=true);

  //beam
  translate([0,0,h]) rotate([90,0,0]) cylinder(h=100,r=r,center=true);

  //screws
  screw_offset = r + r_screw + screw_to_beam;
  translate([+screw_offset,0,0]) cylinder(h=100,r=screw/2,center=true);
  translate([-screw_offset,0,0]) cylinder(h=100,r=screw/2,center=true);
  
  //counter sunk hex
  translate([+screw_offset,0,0]) cylinder(h=hex_depth*2,r=hex_dia/2,center=true, $fn=6);
  translate([-screw_offset,0,0]) cylinder(h=hex_depth*2,r=hex_dia/2,center=true, $fn=6);
}

