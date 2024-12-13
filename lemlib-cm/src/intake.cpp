#include "pros/device.h"
#include "pros/apix.h"

#include "intake.h"
#include "config.h"
#include "pros/motors.h"


struct AnalyzedRing {
  bool valid;
  bool faultyRing;
};
AnalyzedRing analyze_ring(pros::vision_object_s_t ring, int height) {
  pros:printf("height: %d \n", ring.height);

  AnalyzedRing result;
  result.valid = ring.height > height;
  result.faultyRing = ring.height < -10;

  return result;
}

bool Intake::use_sorting = false;
bool Intake::auto_run = false;
bool Intake::auto_reverse = false;
int Intake::signature_id = 0;

Intake::Intake() {
  pros::c::v5_device_e_t vision_sensor_connected = pros::c::registry_get_plugged_type(17);

  if(vision_sensor_connected) {
    use_sorting = true;
  }

};



void Intake::set_team_alliance(std::string team) {
  if(team == "red" || team == "skills") {
    signature_id = 2;
  } else if(team == "blue") {
    signature_id = RED_SIGNATURE;
  }
};

void Intake::run_auto_reverse() {
  auto_run = true;
  auto_reverse = true;
}
void Intake::run_auto() {
  auto_run = true;
}
void Intake::run() {
  intake_chain.move(127);
  intake_stage1.move(-127);
}
void Intake::stop() {
  intake_chain.brake();
  intake_stage1.brake();
  auto_run = false;
  auto_reverse = false;
}
void Intake::reverse() {
  intake_chain.move(-127);
  intake_stage1.move(127);
}

void Intake::task() {
  bool throw_ring = false;

  int ring_not_seen = 0;

  while(true) {
    pros::delay(20);

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || auto_run) {
      if(auto_reverse) {
        Intake::reverse();
        continue;
      }
      Intake::run();


      pros::vision_object_s_t ring = vision_sensor.get_by_sig(0, signature_id);
    
      pros::lcd::print(4, "WH: %d -- %d", ring.width, ring.height);
      // pros::lcd::print(4, "TP: %d -- %d", ring.top_coord, ring.top_coord - ring.height);

      AnalyzedRing ring_analysis = analyze_ring(ring, 105);
      if(ring_analysis.faultyRing) {
        use_sorting = false;
      }

      if(ring_analysis.valid) {
        pros::lcd::print(5, "OBJECT DETECTED");

        ring_not_seen = 0;
        throw_ring = true;
      } else {
        pros::lcd::print(5, "-");
        ring_not_seen++;

        if(throw_ring && ring_not_seen >= 5) {
          intake_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
          Intake::stop();
          pros::delay(500);
          Intake::run();


          throw_ring = false;
          ring_not_seen = 0;
        }
      }
      
    } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) || auto_reverse) {
      Intake::reverse();
    } else {
      Intake::stop();
    }
  }
}