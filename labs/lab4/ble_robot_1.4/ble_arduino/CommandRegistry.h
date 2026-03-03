#ifndef COMMAND_REGISTRY_H
#define COMMAND_REGISTRY_H

#define MAX_COMMANDS 32

typedef void (*CommandHandler)();

extern CommandHandler command_handlers[MAX_COMMANDS];
extern int num_commands;

inline void register_command(
  int cmd_type,
  CommandHandler handler
) {
  if (cmd_type >= 0 && cmd_type < MAX_COMMANDS) {
    command_handlers[cmd_type] = handler;
    if (cmd_type >= num_commands) {
      num_commands = cmd_type + 1;
    }
  }
}

inline void dispatch_command(int cmd_type) {
  if (cmd_type >= 0 && cmd_type < MAX_COMMANDS &&
      command_handlers[cmd_type] != nullptr) {
    command_handlers[cmd_type]();
  } else {
    Serial.print(F("Invalid Command Type: "));
    Serial.println(cmd_type);
  }
}

#endif // COMMAND_REGISTRY_H
