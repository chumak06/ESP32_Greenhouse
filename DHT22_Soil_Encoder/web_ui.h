#pragma once

// Веб-страница: HTML, CSS и JS одним куском. Массив лежит во flash (PROGMEM),
// поэтому не занимает драгоценную RAM. handleRoot() подставляет вместо
// %TOKEN% текущий CSRF-токен, а не отдаёт страницу как есть.
// Правка этого файла требует перепрошивки платы.

const char INDEX_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="ru"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 · датчики</title>
<style>
:root{--bg:#0f1419;--card:#1a2129;--fg:#e6edf3;--dim:#8b98a5;--ok:#3fb950;--bad:#f85149}
*{box-sizing:border-box}
body{margin:0;padding:20px;background:var(--bg);color:var(--fg);font:16px/1.45 system-ui,"Segoe UI",Arial,sans-serif}
h1{margin:0 0 16px;font-size:17px;font-weight:600;color:var(--dim)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;max-width:760px}
.card{background:var(--card);border-radius:12px;padding:14px 16px}
.k{font-size:12px;letter-spacing:.04em;text-transform:uppercase;color:var(--dim)}
.v{margin-top:4px;font-size:30px;font-weight:600;line-height:1.15}
.v small{font-size:15px;font-weight:400;color:var(--dim)}
.on{color:var(--ok)}.off{color:var(--dim)}.err{color:var(--bad)}
.btns{display:flex;gap:6px;margin-top:12px}
.btn{flex:1;padding:8px 4px;border:1px solid #2d3843;border-radius:8px;background:#222b34;
     color:var(--fg);font:14px system-ui,Arial,sans-serif;cursor:pointer}
.btn:hover{background:#2b3541}
.btn.on{border-color:var(--ok);color:var(--ok)}
.btn.off{border-color:var(--bad);color:var(--bad)}
footer{margin-top:16px;color:var(--dim);font-size:13px}
</style></head><body>
<h1>ESP32 · датчики и климат</h1>
<div class="grid">
<div class="card"><div class="k">Температура</div><div class="v" id="t">—</div></div>
<div class="card"><div class="k">Влажность воздуха</div><div class="v" id="h">—</div></div>
<div class="card"><div class="k">Влажность почвы</div><div class="v" id="s">—</div></div>
<div class="card"><div class="k">Освещённость</div><div class="v" id="l">—</div></div>
<div class="card"><div class="k">Помпа</div><div class="v" id="p">—</div>
<div class="btns">
<button class="btn on" onclick="setPump('on')" title="Включить вручную на 60 секунд">ВКЛ</button>
<button class="btn off" onclick="setPump('off')" title="Выключить и не поливать 30 минут">ВЫКЛ</button>
<button class="btn" onclick="setPump('auto')" title="Вернуть автоматический режим">АВТО</button>
</div></div>
<div class="card"><div class="k">Время</div><div class="v" id="tm">—</div></div>
<div class="card"><div class="k">Дата</div><div class="v" id="dt">—</div></div>
</div>
<footer id="st">подключение…</footer>
<script>
let TOKEN = "%TOKEN%";
const $ = id => document.getElementById(id);
const fmtUp = s => s < 3600 ? Math.floor(s / 60) + " мин"
                            : Math.floor(s / 3600) + " ч " + Math.floor(s % 3600 / 60) + " мин";
const fmtLeft = s => s >= 60 ? Math.floor(s / 60) + " мин" : s + " с";
function pumpHtml(d) {
  let s = d.pump ? '<span class="on">ВКЛ</span>' : '<span class="off">выкл</span>';
  let sub = "авто";
  if (d.mode === "on")
    sub = "ручной · " + d.left + " с";
  else if (d.mode === "off")
    sub = "ручной ВЫКЛ · " + fmtLeft(d.left);
  else if (d.ws === "on")
    sub = "полив · " + d.wleft + " с · цикл " + d.cyc;
  else if (d.ws === "pause")
    sub = "пауза · " + d.wleft + " с · замер";
  s += '<div class="k" style="margin-top:6px">' + sub + "</div>";
  return s;
}
async function setPump(state) {
  if (state === "on" && !confirm("Включить помпу вручную на 60 секунд?")) return;
  try {
    const r = await fetch("/pump", {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: "state=" + encodeURIComponent(state) + "&token=" + encodeURIComponent(TOKEN)
    });
    if (!r.ok) throw new Error(r.status + " " + (await r.text()));
    await tick();
  } catch (e) {
    $("st").textContent = "помпа: " + e.message + " — обновите страницу";
  }
}
async function tick() {
  try {
    const r = await fetch("/api", { cache: "no-store" });
    const d = await r.json();
    // Токен приходит заново при каждом опросе: если ESP32 перезагрузилась
    // после открытия страницы, команда всё равно пройдёт.
    if (d.token) TOKEN = d.token;
    $("t").innerHTML = d.t_ok ? d.t.toFixed(1) + "<small> °C</small>" : '<span class="err">ошибка</span>';
    $("h").innerHTML = d.t_ok ? d.h.toFixed(1) + "<small> %</small>" : '<span class="err">ошибка</span>';
    $("s").innerHTML = d.soil_ok
      ? d.soil + "<small> %</small><div class=\"k\" style=\"margin-top:6px\">" + d.mv + " мВ</div>"
      : '<span class="err">датчик</span>';
    $("l").innerHTML = d.light + "<small> %</small><div class=\"k\" style=\"margin-top:6px\">" +
                       d.light_mv + " мВ</div>";
    $("p").innerHTML = pumpHtml(d);
    $("tm").textContent = d.time;
    $("dt").textContent = d.date + " " + d.wd;
    $("st").textContent = "IP " + d.ip + " · RSSI " + d.rssi + " dBm · работает " + fmtUp(d.up);
  } catch (e) {
    $("st").textContent = "нет связи с ESP32";
  }
}
tick(); setInterval(tick, 2000);
</script></body></html>)html";
