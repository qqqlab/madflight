//Motor Mount 3xM2 3x10mm

//=======================================================================
// Config
//=======================================================================
//main cross beam 
beam1_dia = 6.5; //diameter
beam1_depth = 2*beam1_dia; //socket depth
beam1_start = 6; //socket starting point from center of motor
beam1_square = 0S; //beam type: 0=round, 1=square

//outer square beam
beam2_dia = 0; //diameter (0 to ignore)
beam2_depth = 2*beam2_dia; //socket depth
beam2_start = 8.5; //socket starting point from center of motor

//lower cross beam
beam3_dia = 0; //diameter (0 to ignore)
beam3_depth = 2*beam3_dia; //socket depth
beam3_start = 6; //socket starting point from center of motor

//landing beam
beam4_dia = 2.5; //diameter (0 to ignore)
beam4_depth = 2*beam4_dia; //socket depth
beam4_start = 5.25; //socket starting point from center of motor
beam4_start_z = 1; //socket z starting point

//wall thickness for beam sockets
beam_wall = 1;

//Motor Mount 3xM2 3x10mm
//motor screw drill hole diameter
motor_dr1 = 2.3;
//motor screw counterbored hole diameter
motor_dr2 = 5.5;
//thickness motor mounting plate
motor_h = 2.8;
//diameter motor mounting plate
motor_dia = 17;
//motor axis counterbored hole depth
motor_c_depth = 2.8;
//motor axis counterboredhole diameter
motor_c_dia = 6;
//=======================================================================

$fn=60; //number of facets

//outside diameters
beam1_od = beam1_dia+2*beam_wall;
beam2_od = beam2_dia+2*beam_wall;
beam3_od = beam3_dia+2*beam_wall;
beam4_od = beam4_dia+2*beam_wall;

difference()
{
  union() //this union doesn't do anything, just here to make debugging easy
  hull() //comment out hull() for debugging
  {
    //motor mounting plate
    cylinder(h=motor_h,d=motor_dia);
    //sockets for beams
    translate([0,0,beam1_od/2]) rotate([90,0,0]) translate([0,0,beam1_start]) {
      if(beam1_square)
         translate([-beam1_od/2,-beam1_od/2,0]) cube([beam1_od,beam1_od,20]);
      else 
        cylinder(h=beam1_depth,d=beam1_od);
    }
    if(beam2_dia>0) {
      translate([0,0,beam2_od/2]) rotate([90,0,45]) translate([0,0,beam2_start]) cylinder(h=beam2_depth,d=beam2_od);
      translate([0,0,beam2_od/2]) rotate([90,0,-45]) translate([0,0,beam2_start]) cylinder(h=beam2_depth,d=beam2_od);
    }
    if(beam3_dia>0)
      translate([0,0,beam1_od+beam3_od/2-2*beam_wall]) rotate([78,0,0]) translate([0,0,beam3_start]) cylinder(h=beam3_depth,d=beam3_od);
    if(beam4_dia>0)
      translate([0,-beam4_start,0]) rotate([0,0,0]) translate([0,0,beam4_start_z]) cylinder(h=beam4_depth,d=beam4_od);
  }
  
  //drill holes for beams
  translate([0,0,beam1_od/2]) rotate([90,0,0]) translate([0,0,beam1_start]) {
    if(beam1_square)
      translate([-beam1_dia/2,-beam1_dia/2,0]) cube([beam1_dia,beam1_dia,200]);
    else
      cylinder(h=200,d=beam1_dia);
  }
  if(beam2_dia>0) {
    translate([0,0,beam2_od/2]) rotate([90,0,45]) translate([0,0,beam2_start]) cylinder(h=999,d=beam2_dia);
    translate([0,0,beam2_od/2]) rotate([90,0,-45]) translate([0,0,beam2_start]) cylinder(h=999,d=beam2_dia);
  }
  if(beam3_dia>0)
    translate([0,0,beam1_od+beam3_od/2-2*beam_wall]) rotate([78,0,0]) translate([0,0,beam3_start]) cylinder(h=999,d=beam3_dia);
  if(beam4_dia>0)
     translate([0,-beam4_start,0]) rotate([0,0,0]) translate([0,0,beam4_start_z]) cylinder(h=999,d=beam4_dia);
 
  //drill holes for motor mount
  rotate([180,0,30]) DrillMotor3x10();
}

//drill holes for M2 3x10mm motor mount (hole radius 6.666mm)
module DrillMotor3x10(){
  //motor axis hole
  translate([0,0,+50-motor_c_depth]) cylinder(h=100,d=motor_c_dia,center=true);
  //screws
  rotate([0,0,   0]) translate([-10/sqrt(3),0,0]) DrillMotorScrew();
  rotate([0,0, 120]) translate([-10/sqrt(3),0,0]) DrillMotorScrew();
  rotate([0,0,-120]) translate([-10/sqrt(3),0,0]) DrillMotorScrew();  
}

//counterbored hole 
module DrillMotorScrew() {
  translate([0,0,+10]) cylinder(h=999,d=motor_dr1,center=true);
  translate([0,0,-50-motor_h]) cylinder(h=100,d=motor_dr2,center=true);
}