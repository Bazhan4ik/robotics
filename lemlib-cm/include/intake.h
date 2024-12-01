#include <string>


struct AutoIntake {
  bool run;
  bool reverse;
};

class Intake {

private:
  bool allow_intake;


  static bool auto_run;
  static bool auto_reverse;

  static bool use_sorting;
  static int signature_id;

public:

  Intake();

  void run_auto();

  static void run();
  static void stop();
  static void reverse();

  static void set_team_alliance(std::string team);

  static void task();
  
};