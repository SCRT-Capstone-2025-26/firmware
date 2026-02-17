#include <atomic>
#include <SdFat.h>
#include <pico/platform.h>
#include <variant>
// TODO: Possibly add radio

#include "logging.h"
#include "eventqueue.h"
#include "pins.h"
#include "util.h"

// This file handles the code that runs on the other core and handles the logging for Beavs

// See https://stackoverflow.com/questions/64017982/c-equivalent-of-rust-enums
// This allows rust like enums with the c++ variant
// https://en.cppreference.com/w/cpp/utility/variant/visit
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
// explicit deduction guide (not needed as of C++20)
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

template <typename Val, typename... Ts>
auto match(Val &&val, Ts... ts) {
    return std::visit(overloaded{std::forward<Ts>(ts)...}, std::forward<Val>(val));
}

std::atomic_bool log_booted(false);
std::atomic<bool> sd_failure;

SdFs sd;
FsFile log_file;
FsFile data_file;

// This is a class to put logs in the queue from the main core
struct LogEvent {
  Millis timestamp;
  uint core;
  Message value;
};

struct DataEvent {
  Millis timestamp;
  Reading reading;
};

// This is thread safe to store the events put in the queue
// Should be big enough for boot events to build up before being cleared
EventQueue<std::variant<LogEvent, DataEvent>, 64> events;
// Is set to time if there is a log and events is full
// If there are two fails only one is guarranteed to work
std::atomic_bool event_write_fail;

// This can be called from either core and is the main logging functionality
void log_message(Message &&content) {
  if (!events.putQ(LogEvent{millis(), get_core_num(), content})) {
    // If we fail to write then we mark that
    event_write_fail = true;
  }
}

// This should be call by the main core
void write_readings(Reading &&reading) {
  if (!events.putQ(DataEvent{millis(), reading})) {
    // If we fail to write then we mark that
    event_write_fail = true;
  }
}

// This should be called from the other core to confirm that this core has booted
// It returns when this core has booted and returns whether or not the SD is available
bool wait_log_boot() {
  // There are fancier ways, but it doesn't matter since this should be a short wait
  while (!log_booted) { delay(1); }

  return sd_failure;
}

void setup1() {
#ifdef DEBUG
  // Allow some time for the serial to connect
  delay(DEBUG_BOOT_DELAY);
#endif

  // Init the serial
  Serial.begin(115200);
  log_message("Serial inited");

  // Try to init the file we just assume that the file is not inited
  //  until the files are created and written to
  bool file_inited = false;
  // Check if we can access the sd
  // TODO: There is probably some errors that are not being checked (like the returns from mkdir)
  if (sd.begin(SdioConfig(SD_CLOCK, SD_CMD, SD_DATA_0))) {
    log_message("SD inited");

    // Create the log folders if they don't already exist
    sd.mkdir("Logs");
    sd.mkdir("Data");

    // Try to create the log files we just search for the first two files with an available name
    //  by incrementing the number in the name
    for (int i = 0; i < INT_MAX; i++) {
      String log_path = "Logs/log_" + String(i) + ".txt";
      String data_path = "Data/data_" + String(i) + ".csv";
      // Check that both are available continue the loop if not
      if (sd.exists(log_path) || sd.exists(data_path)) {
        continue;
      }

      log_message("File number " + String(i) + " found");

      // Open the files
      log_file = sd.open(log_path, (oflag_t)(O_CREAT | O_WRITE | O_APPEND));
      data_file = sd.open(data_path, (oflag_t)(O_CREAT | O_WRITE | O_APPEND));

      // Init the csv header
      data_file.println("time,acc x,acc y, acc z,gyro x, gyro y,gyro z");
      data_file.flush();

      // We have created log files
      file_inited = true;
      break;
    }
  } else {
    log_message("SD init failed");
  }

  // If the file isn't inited then there is an SD failure
  sd_failure = !file_inited;
  log_booted = true;
}

// Actually write the log to the serial and file if available
// This could be optimized to prevent copying maybe
void write_log(String content) {
  if (!sd_failure) {
    log_file.println(content);
    log_file.flush();
  }

  Serial.println(content);
}

// Handles a log event converting it into something usable
// This could probably be optimized quite a bit because of the string concat and copying
void handle_log_event(LogEvent event) {
  // Convert the log data into a human readable string
  String content = match(event.value,
    [](String str) { return String(str); },
    [](ModeChange change) { return String(MODE_TO_NAME[change.old] + " -> " + MODE_TO_NAME[change.next]); }
  );

  // For some reason the Arduino examples use this string adding
  // So I guess this is idiomatic
  write_log("[time: " + String(event.timestamp) + "ms, core: " + String(event.core) + "] " + content);
}

void handle_data_event(DataEvent event) {
  if (!sd_failure) {
    String content =
      String(event.timestamp) + "," +
      String(event.reading.acc_axis.x) + "," +
      String(event.reading.acc_axis.y) + "," +
      String(event.reading.acc_axis.z) + "," +
      String(event.reading.gyro_axis.x) + "," +
      String(event.reading.gyro_axis.y) + "," +
      String(event.reading.gyro_axis.z);

    data_file.println(content);
    data_file.flush();
  }
}

// Just empties the log queue
void loop1() {
  std::variant<LogEvent, DataEvent> event;

  while (true) {
    events.getQ(event, true);

    // Check if there was an overflow in the event queue
    if (event_write_fail) {
      // Set this false first to catch more overflows
      event_write_fail = false;

      write_log("Log buffer full.");
    }

    // I don't know why the lambdas are needed
    match(event, [](LogEvent event) { handle_log_event(event); }, [](DataEvent event) { handle_data_event(event); });
  }
}

