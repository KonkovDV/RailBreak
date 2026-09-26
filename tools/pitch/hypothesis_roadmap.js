const fs = require("fs");
const path = require("path");
const pptxgen = require("pptxgenjs");

const demo = JSON.parse(
  fs.readFileSync(path.join(__dirname, "../../docs/solution/fork_demo.json"), "utf8")
);
const at = (t) => demo.rows.find((row) => row.t_s === t);

const ink = "1A1814";
const cream = "F6F1E7";
const amber = "E2A23A";
const green = "8FCB9B";
const rust = "E07A5F";
const card = "F6F1E7";
const panel = "2A2622";

const pres = new pptxgen();
pres.defineLayout({ name: "WIDE", width: 13.333, height: 7.5 });
pres.layout = "WIDE";
pres.author = "RailBreak";
pres.title = "Банк путевых гипотез";
pres.subject = "Roadmap. Not the default route-10 filter.";

const slide = pres.addSlide();
slide.background = { color: ink };

slide.addShape(pres.shapes.RECTANGLE, {
  x: 0, y: 0, w: 0.12, h: 7.5, fill: { color: amber },
});

slide.addText("Банк путевых гипотез", {
  x: 0.5, y: 0.28, w: 12.2, h: 0.52,
  fontFace: "Georgia", fontSize: 36, color: cream, margin: 0, bold: true,
});
slide.addText("Не default-фильтр. Маршрут 10 остаётся одним кольцом.", {
  x: 0.5, y: 0.84, w: 12.2, h: 0.3,
  fontFace: "Calibri", fontSize: 16, color: amber, margin: 0,
});

const cards = [
  {
    title: "Сейчас",
    lines: ["Одна координата s", "Состояние [s, v, k, ba]", "Публикуемый фильтр", "Данные маршрута 10"],
  },
  {
    title: "Прототип",
    lines: ["H = (ветка, s, v, w)", "Вес по невязке к точке ветки", "Синтетическая стрелка", "В ноду не входит"],
  },
  {
    title: "Дальше",
    lines: ["Стрелка", "Общая конечная", "Соседний путь", "Ядро состояния то же"],
  },
];
cards.forEach((item, i) => {
  const x = 0.5 + i * 4.2;
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
    x, y: 1.3, w: 4.0, h: 2.15,
    fill: { color: card }, rectRadius: 0.08,
  });
  slide.addText(item.title, {
    x: x + 0.22, y: 1.44, w: 3.56, h: 0.36,
    fontFace: "Georgia", fontSize: 20, color: ink, margin: 0, bold: true,
  });
  slide.addText(item.lines.map((line, n) => ({
    text: line,
    options: { breakLine: n < item.lines.length - 1 },
  })), {
    x: x + 0.22, y: 1.88, w: 3.56, h: 1.4,
    fontFace: "Calibri", fontSize: 16, color: ink, margin: 0,
  });
});

slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
  x: 0.5, y: 3.62, w: 12.3, h: 2.22,
  fill: { color: panel }, rectRadius: 0.08,
});
slide.addText("Синтетическая стрелка", {
  x: 0.72, y: 3.76, w: 3.4, h: 0.28,
  fontFace: "Georgia", fontSize: 16, color: amber, margin: 0, bold: true,
});

slide.addShape(pres.shapes.LINE, {
  x: 0.78, y: 5.05, w: 1.15, h: 0,
  line: { color: cream, width: 3.5 },
});
slide.addShape(pres.shapes.LINE, {
  x: 1.93, y: 5.05, w: 1.35, h: 0,
  line: { color: green, width: 3.5 },
});
slide.addShape(pres.shapes.LINE, {
  x: 1.93, y: 4.25, w: 1.35, h: 0.8,
  line: { color: rust, width: 3.5 }, flipV: true,
});
slide.addText("40 м общий ствол, затем 20°", {
  x: 0.72, y: 5.28, w: 3.3, h: 0.26,
  fontFace: "Calibri", fontSize: 13, color: cream, margin: 0,
});

const marks = [4, 5, 8].map(at);
const labels = ["на развилке", "через 1 с", "через 4 с"];
marks.forEach((row, i) => {
  const x = 4.35 + i * 2.75;
  slide.addText(`${row.t_s.toFixed(1)} с  ·  ${labels[i]}`, {
    x, y: 3.78, w: 2.6, h: 0.26,
    fontFace: "Calibri", fontSize: 13, color: amber, margin: 0,
  });
  slide.addText(row.w_main.toFixed(3), {
    x, y: 4.15, w: 2.6, h: 0.46,
    fontFace: "Georgia", fontSize: 28, color: green, margin: 0, bold: true,
  });
  slide.addText("прямая", {
    x, y: 4.62, w: 2.6, h: 0.22,
    fontFace: "Calibri", fontSize: 13, color: cream, margin: 0,
  });
  slide.addText(row.w_side.toFixed(3), {
    x, y: 4.92, w: 2.6, h: 0.4,
    fontFace: "Georgia", fontSize: 24, color: rust, margin: 0, bold: true,
  });
  slide.addText("боковая", {
    x, y: 5.34, w: 2.6, h: 0.22,
    fontFace: "Calibri", fontSize: 13, color: cream, margin: 0,
  });
});

slide.addText("В MVP топология фиксирована кольцом, потому что выданные данные относятся к маршруту 10. Следующий слой — банк путевых гипотез для стрелок и депо. Ядро состояния при этом не меняется.", {
  x: 0.5, y: 6.08, w: 12.3, h: 0.95,
  fontFace: "Calibri", fontSize: 16, color: cream, margin: 0,
});

slide.addNotes("Offline prototype only. Weights are the synthetic fork in docs/solution/fork_demo.json. Not a route-10 score.");

pres.writeFile({ fileName: path.join(__dirname, "../../docs/solution/hypothesis_roadmap.pptx") })
  .then(() => console.log("wrote hypothesis_roadmap.pptx"));
