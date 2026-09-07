/* 7-slide jury pitch for tramDR-0.0.11. Run from repo root:
 *   npm exec --yes --package pptxgenjs -- node tools/pitch/build_pitch.js
 */
const path = require("path");
const pptxgen = require("pptxgenjs");

const ROOT = path.resolve(__dirname, "../..");
const OUT = path.join(ROOT, "docs", "pitch.pptx");
const PLOT = (name) => path.join(ROOT, "evidence", "plots-pitch", name);

const C = {
  night: "0B1C2C",
  ink: "1A2330",
  paper: "F4F1EA",
  brass: "C4A35A",
  steel: "5C6B78",
  muted: "4A5560",
  white: "F7F5F0",
  card: "FFFFFF",
  danger: "B42318",
  ok: "1F6B4A",
  line: "D9D2C5",
};

function shadow() {
  return { type: "outer", color: "000000", blur: 8, offset: 2, angle: 135, opacity: 0.12 };
}

function footer(slide, dark) {
  slide.addText("tramDR-0.0.11  ·  маршрут 10  ·  без GNSS в фильтре", {
    x: 0.5, y: 5.28, w: 9.0, h: 0.22,
    fontFace: "Calibri", fontSize: 10, color: dark ? "8A9AAB" : C.steel, margin: 0,
  });
}

async function main() {
  const pres = new pptxgen();
  pres.defineLayout({ name: "JURY", width: 10, height: 5.625 });
  pres.layout = "JURY";
  pres.author = "Коныков Д.В.";
  pres.title = "tramDR — резервная одометрия без GNSS";
  pres.subject = "Хакатон Московского транспорта, задача № 3";

  // 1. Title
  {
    const s = pres.addSlide();
    s.background = { color: C.night };
    s.addShape(pres.shapes.RECTANGLE, {
      x: 0, y: 0, w: 0.18, h: 5.625, fill: { color: C.brass },
    });
    s.addText("ЗАДАЧА № 3", {
      x: 0.55, y: 0.85, w: 8.8, h: 0.28,
      fontFace: "Calibri", fontSize: 12, color: C.brass, bold: true,
      charSpacing: 3, margin: 0,
    });
    s.addText("tramDR", {
      x: 0.55, y: 1.25, w: 8.8, h: 0.85,
      fontFace: "Georgia", fontSize: 54, color: C.white, bold: true, margin: 0,
    });
    s.addText("Резервная одометрия по модели привода и колёс", {
      x: 0.55, y: 2.15, w: 8.8, h: 0.42,
      fontFace: "Calibri", fontSize: 20, color: C.brass, margin: 0,
    });
    s.addText("Маршрут 10  ·  71-911ЕМ «Львёнок-Москва»  ·  GNSS, IMU и лидар не входят в фильтр", {
      x: 0.55, y: 2.7, w: 8.8, h: 0.32,
      fontFace: "Calibri", fontSize: 14, color: "8A9AAB", margin: 0,
    });
    s.addText("Ядро C++17  ·  ROS 2 Humble  ·  tramDR-0.0.11", {
      x: 0.55, y: 4.55, w: 8.8, h: 0.28,
      fontFace: "Calibri", fontSize: 13, color: C.white, margin: 0,
    });
    s.addNotes(
      "5 минут. Не обещать SIL. Главный тезис: физика привода + UKF + явная метрика HMI, не новый абстрактный фильтр.",
    );
  }

  // 2. Problem
  {
    const s = pres.addSlide();
    s.background = { color: C.paper };
    s.addText("Где штатная локализация молчит", {
      x: 0.5, y: 0.28, w: 9.0, h: 0.42,
      fontFace: "Georgia", fontSize: 26, color: C.ink, margin: 0,
    });
    const cards = [
      { t: "Городской каньон", d: "Метель, юз, бедные лидарные признаки. GNSS в каньоне не спасает фильтр — его там нет." },
      { t: "Строгинский мост", d: "Открытая вода, неизвестный уклон 25–40 ‰. i(s) в ноду не кладём: F_bias съедает рампу." },
      { t: "Все оси моторные", d: "Bo-Bo, нет бегунка. Второй принцип одометрии — модель кузова по тяге и тормозу." },
    ];
    cards.forEach((c, i) => {
      const x = 0.5 + i * 3.1;
      s.addShape(pres.shapes.RECTANGLE, {
        x, y: 0.9, w: 2.95, h: 2.55,
        fill: { color: C.card }, shadow: shadow(),
      });
      s.addShape(pres.shapes.RECTANGLE, {
        x, y: 0.9, w: 0.08, h: 2.55, fill: { color: C.brass },
      });
      s.addText(c.t, {
        x: x + 0.22, y: 1.05, w: 2.6, h: 0.55,
        fontFace: "Georgia", fontSize: 16, color: C.ink, bold: true, margin: 0,
      });
      s.addText(c.d, {
        x: x + 0.22, y: 1.65, w: 2.6, h: 1.55,
        fontFace: "Calibri", fontSize: 13, color: C.muted, margin: 0,
      });
    });
    s.addText("Метрика целостности — HMI-rate: доля кадров OK, когда ошибка пути уже выше предела безопасности.", {
      x: 0.5, y: 3.65, w: 9.0, h: 0.4,
      fontFace: "Calibri", fontSize: 14, color: C.ink, margin: 0,
    });
    s.addText("Combino NF100 — синтетический twin. Лидар ЦБТ — только оффлайн GT, не измерение фильтра.", {
      x: 0.5, y: 4.1, w: 9.0, h: 0.32,
      fontFace: "Calibri", fontSize: 13, color: C.steel, margin: 0,
    });
    footer(s, false);
    s.addNotes(
      "Маршрут 10, Львёнок, все оси моторные. Не путать с Витязем. Мост: центр 32 промилле — оценка, не факт. Июнь 2026 — временная линия.",
    );
  }

  // 3. Scheme
  {
    const s = pres.addSlide();
    s.background = { color: C.paper };
    s.addText("Два принципа, независимый чекер", {
      x: 0.5, y: 0.28, w: 9.0, h: 0.4,
      fontFace: "Georgia", fontSize: 26, color: C.ink, margin: 0,
    });
    const cols = [
      { h: "Канал A", b: "Модель кузова\nтяга + тормоз + Дэвис\nF_bias на неизвестный уклон" },
      { h: "Канал B", b: "Колёса ω\nкрип по EN 15595\nне сертификация" },
      { h: "Защита", b: "κ мгновенно\nинтегратор D (WSP)\nLOST, если оба лгут" },
    ];
    cols.forEach((c, i) => {
      const x = 0.5 + i * 3.1;
      s.addShape(pres.shapes.RECTANGLE, {
        x, y: 0.85, w: 2.95, h: 2.15,
        fill: { color: C.night },
      });
      s.addText(c.h, {
        x: x + 0.18, y: 0.98, w: 2.6, h: 0.38,
        fontFace: "Georgia", fontSize: 16, color: C.brass, margin: 0,
      });
      s.addText(c.b, {
        x: x + 0.18, y: 1.42, w: 2.6, h: 1.4,
        fontFace: "Calibri", fontSize: 14, color: C.white, margin: 0,
      });
    });
    s.addText(
      "Стоянка только по свидетельству: 0.32 с нулей на выбеге ≠ ZUPT. Заклиненные колёса не обнуляют движущийся кузов.\ncheck_envelope.py не импортирует UKF. Статус — /tram/diagnostics, не поле Odometry.",
      {
        x: 0.5, y: 3.2, w: 9.0, h: 0.85,
        fontFace: "Calibri", fontSize: 14, color: C.ink, margin: 0,
      },
    );
    footer(s, false);
    s.addNotes(
      "Магниторельс — только по факту включения. Канал A не статистически независим: общие параметры UKF. Это надо сказать самим, если спросят.",
    );
  }

  // 4. Observability
  {
    const s = pres.addSlide();
    s.background = { color: C.paper };
    s.addText("Что видно — и чего нет", {
      x: 0.5, y: 0.28, w: 9.0, h: 0.4,
      fontFace: "Georgia", fontSize: 26, color: C.ink, margin: 0,
    });
    const blocks = [
      { t: "Наблюдаемо", c: C.ok, d: "отношения диаметров\nмасштаб v / d̅\nотношение a / v" },
      { t: "Слабо", c: C.brass, d: "средний диаметр d̅\nмасса отдельно от тяги\nкалибровка identify_coast" },
      { t: "Ненаблюдаемо", c: C.danger, d: "s после полного юза\nF_bias отдельно от уклона\nμ без момента на валу" },
    ];
    blocks.forEach((b, i) => {
      const x = 0.5 + i * 3.1;
      s.addShape(pres.shapes.RECTANGLE, {
        x, y: 0.85, w: 2.95, h: 2.35,
        fill: { color: C.card }, shadow: shadow(),
      });
      s.addShape(pres.shapes.RECTANGLE, {
        x, y: 0.85, w: 2.95, h: 0.08, fill: { color: b.c },
      });
      s.addText(b.t, {
        x: x + 0.18, y: 1.08, w: 2.6, h: 0.38,
        fontFace: "Georgia", fontSize: 16, color: C.ink, margin: 0,
      });
      s.addText(b.d, {
        x: x + 0.18, y: 1.52, w: 2.6, h: 1.45,
        fontFace: "Calibri", fontSize: 14, color: C.muted, margin: 0,
      });
    });
    s.addText("α = 0.58: окно W_c⁰≥0 и Luo–Moroz. Смена β без пересчёта α отвергается. RAIB 08/2026: песок не гарантирует остановку — обязан DEGRADED.", {
      x: 0.5, y: 3.4, w: 9.0, h: 0.7,
      fontFace: "Calibri", fontSize: 13, color: C.ink, margin: 0,
    });
    footer(s, false);
    s.addNotes(
      "mismatch_r0 — это не «врём в половине случаев», а ненаблюдаемый масштаб. identify_coast. Не обещать самокалибровку любой поездки.",
    );
  }

  // 5. Numbers
  {
    const s = pres.addSlide();
    s.background = { color: C.paper };
    s.addText("Синтетика seed 42, ядро 0.0.11", {
      x: 0.5, y: 0.22, w: 5.4, h: 0.38,
      fontFace: "Georgia", fontSize: 22, color: C.ink, margin: 0,
    });
    const stats = [
      { n: "0", l: "HMI на 17 из 18\nсценариев twin" },
      { n: "0.515", l: "mismatch_r0\nненаблюдаемый r₀" },
      { n: "0.14 м", l: "slide_brake\nдо первого DEGRADED" },
    ];
    stats.forEach((st, i) => {
      const x = 0.5 + i * 1.85;
      s.addShape(pres.shapes.RECTANGLE, {
        x, y: 0.7, w: 1.75, h: 1.85,
        fill: { color: C.night },
      });
      s.addText(st.n, {
        x, y: 0.82, w: 1.75, h: 0.7,
        fontFace: "Georgia", fontSize: 22, color: C.brass, align: "center", margin: 0,
      });
      s.addText(st.l, {
        x: x + 0.08, y: 1.52, w: 1.59, h: 0.85,
        fontFace: "Calibri", fontSize: 11, color: C.white, align: "center", margin: 0,
      });
    });
    s.addImage({
      path: PLOT("slide_brake.svg"),
      x: 6.05, y: 0.55, w: 3.45, h: 2.55,
    });
    s.addText("Бурназяна → Исаковского, 2385.6 м без колёс: PL 124.05 против AL 124.28 м. Зазор 0.23 м — совпадение параметров, не проект. Защищает LOST.", {
      x: 0.5, y: 2.7, w: 5.4, h: 0.95,
      fontFace: "Calibri", fontSize: 13, color: C.ink, margin: 0,
    });
    s.addText("Архивное 0.42 — другой клип d≥0.90. Канон 0.515. Не поле, не bag организатора.", {
      x: 0.5, y: 3.7, w: 9.0, h: 0.35,
      fontFace: "Calibri", fontSize: 12, color: C.steel, margin: 0,
    });
    footer(s, false);
    s.addNotes(
      "Произнести PL/AL самим. 1% по диаметру = 23.9 м. Медленнее ехать на необслуживаемом режиме хуже: sigma ~ t^{3/2}. b_s ≈ 0.75 м на twin, 0.83 м при 60 км/ч.",
    );
  }

  // 6. Demo / deploy
  {
    const s = pres.addSlide();
    s.background = { color: C.paper };
    s.addText("Запуск и внедрение", {
      x: 0.5, y: 0.28, w: 9.0, h: 0.4,
      fontFace: "Georgia", fontSize: 26, color: C.ink, margin: 0,
    });
    const rows = [
      { t: "Без ROS", d: "CMake → ctest (8 целей) → generate → replay_ukf → независимый чекер" },
      { t: "С bag", d: "inspect_bag → identify_coast / notch / jerk → run_bag. Сторож 50 Гц не молчит без колёс." },
      { t: "Рядом со штатным", d: "Ядро без ROS. Отдельный контур /tram/*. SIL и CENELEC не заявляются." },
      { t: "Такт", d: "p50 ≈ 10 мкс на этой машине при шаге 20 мс. Не стенд заказчика, не WCET." },
    ];
    rows.forEach((r, i) => {
      const y = 0.85 + i * 0.85;
      s.addShape(pres.shapes.RECTANGLE, {
        x: 0.5, y, w: 9.0, h: 0.75,
        fill: { color: C.card }, shadow: shadow(),
      });
      s.addShape(pres.shapes.RECTANGLE, {
        x: 0.5, y, w: 0.08, h: 0.75, fill: { color: C.brass },
      });
      s.addText(r.t, {
        x: 0.78, y: y + 0.08, w: 2.3, h: 0.58,
        fontFace: "Georgia", fontSize: 15, color: C.ink, valign: "middle", margin: 0,
      });
      s.addText(r.d, {
        x: 3.15, y: y + 0.08, w: 6.1, h: 0.58,
        fontFace: "Calibri", fontSize: 14, color: C.muted, valign: "middle", margin: 0,
      });
    });
    footer(s, false);
    s.addNotes("Демо: пробуксовка, обрыв оси, рост P, DEGRADED/LOST. Не обещать Docker runtime — не гоняли.");
  }

  // 7. Close
  {
    const s = pres.addSlide();
    s.background = { color: C.night };
    s.addShape(pres.shapes.RECTANGLE, {
      x: 0, y: 0, w: 0.18, h: 5.625, fill: { color: C.brass },
    });
    s.addText("Главный тезис", {
      x: 0.55, y: 0.55, w: 8.8, h: 0.3,
      fontFace: "Calibri", fontSize: 12, color: C.brass, bold: true, charSpacing: 2, margin: 0,
    });
    s.addText("Не новый абстрактный фильтр — резервная одометрия на физике привода для маршрута 10, с явной метрикой опасного отказа там, где лидар и GNSS не работают.", {
      x: 0.55, y: 0.95, w: 8.8, h: 1.35,
      fontFace: "Georgia", fontSize: 20, color: C.white, margin: 0,
    });
    s.addText("Говорят, не показывают", {
      x: 0.55, y: 2.5, w: 8.8, h: 0.28,
      fontFace: "Calibri", fontSize: 12, color: C.brass, bold: true, charSpacing: 1, margin: 0,
    });
    s.addText("1. Письмо 14.4–14.6: публичный MIT уже публикация. Текст в NOTICE, отправка — человек.\n2. Bag организатора — с 25.09. Всё измеренное сегодня — синтетический twin.\n3. SIL нет. F-01 не ослабляли: короткий замок нулей не стоянка.", {
      x: 0.55, y: 2.88, w: 8.8, h: 1.35,
      fontFace: "Calibri", fontSize: 15, color: "D5DDE6", margin: 0,
    });
    s.addText("github.com/KonkovDV/RailBreak", {
      x: 0.55, y: 4.55, w: 8.8, h: 0.28,
      fontFace: "Calibri", fontSize: 14, color: C.brass, margin: 0,
    });
    s.addNotes(
      "Если спросят «что не работает» — этот слайд. Письмо отправить сегодня. Репозиторий public; при отказе организатора — private с доступом жюри.",
    );
  }

  await pres.writeFile({ fileName: OUT });
  console.log("wrote", OUT);
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
