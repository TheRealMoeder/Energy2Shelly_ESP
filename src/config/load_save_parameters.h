#ifndef LOAD_SAVE_PARAMETERS_H
#define LOAD_SAVE_PARAMETERS_H

void save_parameters();
void load_parameters();
extern String json_prefs_string;
extern JsonDocument update_config_doc;
void mergeJson(JsonObject target, JsonObjectConst source);

#endif // LOAD_SAVE_PARAMETERS_H