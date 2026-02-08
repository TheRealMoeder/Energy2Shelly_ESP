#ifndef HTML_SAVE_SUCCESS_H
#define HTML_SAVE_SUCCESS_H

#include <Arduino.h>

const char HTML_SAVE_SUCCESS[] PROGMEM = R"=====(
<!DOCTYPE html><html><head><title>Settings Saved</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<link rel='icon' href='/favicon.svg' type='image/svg+xml'>
<style>
body{font-family:Arial,sans-serif;text-align:center;padding:40px;background:#f4f4f4}
.container{max-width:500px;margin:0 auto;background:#fff;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}
h1{color:#4CAF50;margin-bottom:20px}
.spinner{border:4px solid #f3f3f3;border-top:4px solid #4CAF50;border-radius:50%;width:40px;height:40px;animation:spin 1s linear infinite;margin:20px auto}
@keyframes spin{0%{transform:rotate(0deg)}100%{transform:rotate(360deg)}}
.countdown{font-size:48px;color:#4CAF50;font-weight:bold;margin:20px 0}
p{color:#555;line-height:1.6}
a{color:#007bff;text-decoration:none}
</style></head><body>
<div class='container'>
<h1>Settings Saved Successfully!</h1>
<div class='spinner'></div>
<p>Your configuration has been saved.</p>
<p><strong>The device is restarting...</strong></p>
<div class='countdown' id='countdown'>30</div>
<p>Redirecting to home page, when countdown has finished</p>
<p><a href='/'>Click here</a> if not redirected automatically</p>
</div>
<script>
let seconds=30;
const timer=setInterval(()=>{
seconds--;
document.getElementById('countdown').textContent=seconds;
if(seconds<=0){clearInterval(timer);window.location.href='/';}
},1000);
</script>
</body></html>
)=====";

#endif // HTML_SAVE_SUCCESS_H
