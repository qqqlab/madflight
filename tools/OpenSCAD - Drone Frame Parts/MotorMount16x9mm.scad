//Motor Mount 4xM3 16x19mm

//=======================================================================
// Config
//=======================================================================
//main cross beam 
beam1_dia = 6.5; //diameter
beam1_depth = 3*beam1_dia; //socket depth
beam1_start = 13; //socket starting point from center of motor
beam1_square = 0; //beam type: 0=round, 1=square
beam1_tightener = 2; //add screw tightener: 0=none(i.e. use glue) 1=horizonal 2=vertical

//outer square beam
beam2_dia = 3.5; //diameter (0 to ignore)
beam2_depth = 0; //socket depth
beam2_start = 6; //socket starting point from center of motor

//lower cross beam
beam3_dia = 0; //diameter (0 to ignore)
beam3_depth = 3*beam3_dia; //socket depth
beam3_start = 11; //socket starting point from center of motor

//landing beams
beam4_dia = 3.5; //diameter (0 to ignore)
beam4_depth = 0; //socket depth
beam4_start = 15; //socket starting point from center of motor
beam4_start_z = 1.5; //socket z starting point

//wall thickness for beam sockets
beam_wall = 1;

//Motor Mount 3xM2 3x10mm
//motor screw drill hole diameter
motor_dr1 = 3.3;
//motor screw counterbored hole diameter
motor_dr2 = 7;
//thickness motor mounting plate
motor_h = 2.8;
//diameter motor mounting plate
motor_dia = 28;
//motor axis counterbored hole depth
motor_c_depth = 4;
//motor axis counterbored hole diameter
motor_c_dia = 9;

//=======================================================================

$fn=60; //number of facets

//outside diameters
beam1_od = beam1_dia+2*beam_wall;
beam2_od = beam2_dia+2*beam_wall;
beam3_od = beam3_dia+2*beam_wall;
beam4_od = beam4_dia+2*beam_wall;

difference() {
  union() {
    MotorMount_add();
    beam1_tigthener_add();
  }
  MotorMount_drill();
  beam1_tigthener_drill();
}

module MotorMount_add() {
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
    if(beam4_dia>0) {
      translate([(beam1_dia+beam4_dia)/2,-beam4_start,0]) rotate([0,0,0]) translate([0,0,beam4_start_z]) cylinder(h=beam4_depth,d=beam4_od);
      translate([-(beam1_dia+beam4_dia)/2,-beam4_start,0]) rotate([0,0,0]) translate([0,0,beam4_start_z]) cylinder(h=beam4_depth,d=beam4_od);
    }
  }
}
module MotorMount_drill() {
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
  if(beam4_dia>0) {
     translate([(beam1_dia+beam4_dia)/2,-beam4_start,0]) rotate([0,0,0]) translate([0,0,beam4_start_z]) cylinder(h=999,d=beam4_dia);
     translate([-(beam1_dia+beam4_dia)/2,-beam4_start,0]) rotate([0,0,0]) translate([0,0,beam4_start_z]) cylinder(h=999,d=beam4_dia);
  }
 
  //drill holes for motor mount
  rotate([180,0,0]) DrillMotor16x19();
}

//drill holes for M3 16x19mm motor mount
module DrillMotor16x19(){
  //motor axis hole
  translate([0,0,+50-motor_c_depth]) cylinder(h=100,d=motor_c_dia,center=true);
  //screws
  rotate([0,0, 90]) translate([-8,0,0]) DrillMotorScrew();
  rotate([0,0,-90]) translate([-8,0,0]) DrillMotorScrew();
  rotate([0,0,  0]) translate([-9.5,0,0]) DrillMotorScrew();
  rotate([0,0,180]) translate([-9.5,0,0]) DrillMotorScrew();  
}

//counterbored hole
module DrillMotorScrew() {
  translate([0,0,+10]) cylinder(h=999,d=motor_dr1,center=true);
  translate([0,0,-50-motor_h]) cylinder(h=100,d=motor_dr2,center=true);
}

module nutholeM3() {
  cylinder(r= 6.4/2, h=100, $fn=6, center=[0,0]);
  translate([0,0,-99]) cylinder(r= 3.3/2, h=100, center=[0,0]);
}

//beam1 tigthener
module beam1_tigthener_add() {
  if(beam1_tightener==1) {
    w = beam1_dia + 10;
    d = 8;
    h = 8;
    translate([-w/2,-beam1_start-beam1_depth,-h/2+beam1_od/2]) cube([w, d, h]);
  }
  if(beam1_tightener==2) {
    w = 10;
    d = 8;
    h1 = 6;
    h = beam1_dia/2+h1;
    translate([-w/2, -beam1_start-beam1_depth, +beam1_od/2]) cube([w, d, h]);
  }
}
module beam1_tigthener_drill() {
  if(beam1_tightener==1) {
    w = beam1_dia + 10;
    d = 8;
    h = 8;
    translate([+beam1_dia/2+2,-beam1_start-beam1_depth+d/2,-h/2+beam1_od/2+h-1.5]) rotate([0,0,90]) nutholeM3();
    translate([-beam1_dia/2-2,-beam1_start-beam1_depth+d/2,-h/2+beam1_od/2+h-1.5]) rotate([0,0,90]) nutholeM3();
    //gap
    h2 = 1.5; //gap width
    d2 = beam1_depth/2;
    translate([-50,-beam1_start-beam1_depth-0.01,-h2/2+beam1_od/2]) cube([100, d2, h2]);
  }
  if(beam1_tightener==2) {
      w = 10;
      d = 8;
      h1 = 6;
      translate([w/2-1.5, -beam1_start-beam1_depth+d/2, +beam1_od/2+beam1_dia/2+h1/2]) rotate([90,0,90]) nutholeM3();
      //gap
      w2 = 1.5; //gap width
      d2 = beam1_depth/2;
      h2 = beam1_dia/2;
      translate([-w2/2,-beam1_start-beam1_depth-0.01,-h2/2+beam1_od/2]) cube([w2, d2, 100]);
  }
}