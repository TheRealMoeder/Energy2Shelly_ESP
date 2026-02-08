#ifndef HTML_RESET_H
#define HTML_RESET_H

#include <Arduino.h>

const char HTML_RESET[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<title>Reset Confirmation</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<link rel='icon' href='/favicon.svg' type='image/svg+xml'>
<style>
  body { font-family: Arial, sans-serif; text-align: center; padding: 20px; }
  .btn { padding: 10px 20px; margin: 10px; cursor: pointer; text-decoration: none; display: inline-block; border-radius: 5px; font-size: 16px; }
  .btn-yes { background-color: #d9534f; color: white; border: none; }
  .btn-no { background-color: #5bc0de; color: white; border: none; }
</style>
</head>
<body>
  <h2>Reset Configuration?</h2>
  <p>Are you sure you want to reset the WiFi configuration? This will clear all settings and restart the device.</p>
  <form method='POST' action='/reset' style='display:inline;'>
    <button type='submit' class='btn btn-yes'>Yes, Reset</button>
  </form>
  <a href='/' class='btn btn-no'>Cancel</a>
</body>
</html>
)=====";

#endif // HTML_RESET_H
