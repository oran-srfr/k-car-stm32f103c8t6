from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.platypus import (
    BaseDocTemplate,
    PageTemplate,
    Frame,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
    LongTable,
    PageBreak,
    KeepTogether,
    HRFlowable,
)
from reportlab.graphics.shapes import Drawing, Rect, Circle, String, Line, Polygon


OUTPUT_PDF = r"C:\Users\29579\OneDrive\文档\ChatGPT\校电赛\output\pdf\K题场景感知循迹小车采购清单.pdf"


# Fonts
pdfmetrics.registerFont(
    TTFont("YaHei", r"C:\Windows\Fonts\msyh.ttc", subfontIndex=0)
)
pdfmetrics.registerFont(
    TTFont("YaHeiB", r"C:\Windows\Fonts\msyhbd.ttc", subfontIndex=0)
)
pdfmetrics.registerFontFamily(
    "YaHei",
    normal="YaHei",
    bold="YaHeiB",
    italic="YaHei",
    boldItalic="YaHeiB",
)


# Palette
NAVY = colors.HexColor("#17324D")
BLUE = colors.HexColor("#2E6E9E")
TEAL = colors.HexColor("#1F7A73")
LIGHT_BLUE = colors.HexColor("#EAF2F8")
PALE = colors.HexColor("#F6F8FA")
LINE = colors.HexColor("#D9E2EC")
GRAY = colors.HexColor("#5F6B7A")
MUTED = colors.HexColor("#7B8794")
RED = colors.HexColor("#B03A2E")
ORANGE = colors.HexColor("#C26A1B")
GREEN = colors.HexColor("#2E7D5B")
WHITE = colors.white


# Page geometry
PAGE_W, PAGE_H = A4
LEFT = 17 * mm
RIGHT = 17 * mm
TOP = 22 * mm
BOTTOM = 17 * mm
CONTENT_W = PAGE_W - LEFT - RIGHT


# Styles
title_small = ParagraphStyle(
    "title_small",
    fontName="YaHei",
    fontSize=8.6,
    leading=11,
    textColor=colors.HexColor("#BFD7EA"),
)
title_main = ParagraphStyle(
    "title_main",
    fontName="YaHeiB",
    fontSize=25,
    leading=30,
    textColor=WHITE,
)
title_sub = ParagraphStyle(
    "title_sub",
    fontName="YaHei",
    fontSize=13,
    leading=17,
    textColor=WHITE,
)
title_meta = ParagraphStyle(
    "title_meta",
    fontName="YaHei",
    fontSize=8.1,
    leading=10.5,
    textColor=colors.HexColor("#D6E4F0"),
)
section = ParagraphStyle(
    "section",
    fontName="YaHeiB",
    fontSize=13,
    leading=17,
    textColor=NAVY,
)
subsection = ParagraphStyle(
    "subsection",
    fontName="YaHeiB",
    fontSize=10.2,
    leading=13.5,
    textColor=TEAL,
)
body = ParagraphStyle(
    "body",
    fontName="YaHei",
    fontSize=8.9,
    leading=13.8,
    textColor=colors.HexColor("#26323D"),
    wordWrap="CJK",
)
body_small = ParagraphStyle(
    "body_small",
    fontName="YaHei",
    fontSize=8.2,
    leading=11.5,
    textColor=colors.HexColor("#26323D"),
    wordWrap="CJK",
)
note = ParagraphStyle(
    "note",
    fontName="YaHei",
    fontSize=7.7,
    leading=10.2,
    textColor=GRAY,
    wordWrap="CJK",
)
summary_value = ParagraphStyle(
    "summary_value",
    fontName="YaHeiB",
    fontSize=12.4,
    leading=15,
    textColor=NAVY,
    alignment=TA_CENTER,
)
summary_label = ParagraphStyle(
    "summary_label",
    fontName="YaHei",
    fontSize=7.3,
    leading=9.2,
    textColor=GRAY,
    alignment=TA_CENTER,
)
table_header = ParagraphStyle(
    "table_header",
    fontName="YaHeiB",
    fontSize=7.6,
    leading=9.3,
    textColor=WHITE,
    alignment=TA_CENTER,
)
table_cell = ParagraphStyle(
    "table_cell",
    fontName="YaHei",
    fontSize=7.15,
    leading=9.15,
    textColor=colors.HexColor("#1F2A33"),
    wordWrap="CJK",
)
table_cell_center = ParagraphStyle(
    "table_cell_center",
    parent=table_cell,
    alignment=TA_CENTER,
)
table_item = ParagraphStyle(
    "table_item",
    parent=table_cell,
    fontName="YaHeiB",
)
priority_buy = ParagraphStyle(
    "priority_buy",
    parent=table_cell_center,
    fontName="YaHeiB",
    textColor=RED,
)
priority_rec = ParagraphStyle(
    "priority_rec",
    parent=table_cell_center,
    fontName="YaHeiB",
    textColor=BLUE,
)
priority_opt = ParagraphStyle(
    "priority_opt",
    parent=table_cell_center,
    textColor=MUTED,
)
priority_no = ParagraphStyle(
    "priority_no",
    parent=table_cell_center,
    fontName="YaHeiB",
    textColor=ORANGE,
)
check_style = ParagraphStyle(
    "check_style",
    parent=table_cell_center,
    fontSize=10.5,
    leading=12,
)
group_style = ParagraphStyle(
    "group_style",
    fontName="YaHeiB",
    fontSize=8.7,
    leading=11,
    textColor=NAVY,
)


def p(text, style=body):
    return Paragraph(text, style)


def section_header(number, title, subtitle=None):
    items = [
        Spacer(1, 3.2 * mm),
        Paragraph(
            f'<font color="#2E6E9E">{number}</font>  {title}',
            section,
        ),
        HRFlowable(
            width=CONTENT_W,
            thickness=0.8,
            color=LINE,
            spaceBefore=1.2 * mm,
            spaceAfter=2.2 * mm,
        ),
    ]
    if subtitle:
        items.append(Paragraph(subtitle, note))
        items.append(Spacer(1, 1.6 * mm))
    return items


def callout(text, bg=LIGHT_BLUE, border=BLUE, text_style=body_small):
    t = Table([[Paragraph(text, text_style)]], colWidths=[CONTENT_W])
    t.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), bg),
                ("BOX", (0, 0), (-1, -1), 0.6, border),
                ("LEFTPADDING", (0, 0), (-1, -1), 7),
                ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                ("TOPPADDING", (0, 0), (-1, -1), 4.5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4.5),
            ]
        )
    )
    return t


def group_header(label, note_text=""):
    cells = [Paragraph(label, group_style)]
    if note_text:
        cells.append(
            Paragraph(
                note_text,
                ParagraphStyle(
                    "group_note",
                    parent=note,
                    alignment=TA_LEFT,
                    textColor=GRAY,
                ),
            )
        )
    widths = [CONTENT_W * 0.68, CONTENT_W * 0.32] if note_text else [CONTENT_W]
    t = Table([cells], colWidths=widths)
    t.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), LIGHT_BLUE),
                ("LINEBELOW", (0, 0), (-1, -1), 0.4, LINE),
                ("LEFTPADDING", (0, 0), (-1, -1), 6),
                ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                ("TOPPADDING", (0, 0), (-1, -1), 4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
            ]
        )
    )
    return t


def priority_para(priority):
    mapping = {
        "必买": priority_buy,
        "推荐": priority_rec,
        "可选": priority_opt,
        "不建议": priority_no,
    }
    return Paragraph(priority, mapping.get(priority, table_cell_center))


def make_bom_table(rows):
    header = [
        p("序号", table_header),
        p("物品", table_header),
        p("建议规格 / 说明", table_header),
        p("数量", table_header),
        p("参考价", table_header),
        p("优先级", table_header),
        p("采购", table_header),
    ]
    data = [header]
    for rid, name, spec, qty, price, prio in rows:
        data.append(
            [
                p(rid, table_cell_center),
                p(name, table_item),
                p(spec, table_cell),
                p(qty, table_cell_center),
                p(price, table_cell_center),
                priority_para(prio),
                p("□", check_style),
            ]
        )
    widths = [
        8.5 * mm,
        30.5 * mm,
        76.0 * mm,
        13.0 * mm,
        24.0 * mm,
        14.5 * mm,
        7.5 * mm,
    ]
    t = LongTable(data, colWidths=widths, repeatRows=1)
    style = [
        ("BACKGROUND", (0, 0), (-1, 0), NAVY),
        ("TEXTCOLOR", (0, 0), (-1, 0), WHITE),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ALIGN", (0, 0), (-1, 0), "CENTER"),
        ("ALIGN", (0, 1), (0, -1), "CENTER"),
        ("ALIGN", (3, 1), (6, -1), "CENTER"),
        ("LEFTPADDING", (0, 0), (-1, -1), 2.2),
        ("RIGHTPADDING", (0, 0), (-1, -1), 2.2),
        ("TOPPADDING", (0, 0), (-1, -1), 2.0),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1.8),
        ("LINEBELOW", (0, 0), (-1, -1), 0.35, LINE),
        ("BOX", (0, 0), (-1, -1), 0.45, LINE),
    ]
    for i in range(1, len(data)):
        if i % 2 == 0:
            style.append(("BACKGROUND", (0, i), (-1, i), PALE))
    t.setStyle(TableStyle(style))
    return t


class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        self._saved_page_states = []
        canvas.Canvas.__init__(self, *args, **kwargs)

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        page_count = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self._draw_footer(page_count)
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)

    def _draw_footer(self, page_count):
        self.saveState()
        self.setStrokeColor(LINE)
        self.setLineWidth(0.45)
        self.line(LEFT, 12.5 * mm, PAGE_W - RIGHT, 12.5 * mm)
        self.setFont("YaHei", 7.2)
        self.setFillColor(GRAY)
        self.drawString(LEFT, 8.6 * mm, "K题 · 场景感知循迹小车采购清单")
        self.drawRightString(
            PAGE_W - RIGHT,
            8.6 * mm,
            f"第 {self._pageNumber} 页 / 共 {page_count} 页",
        )
        self.restoreState()


def on_later_pages(canv, doc):
    canv.saveState()
    canv.setFont("YaHeiB", 7.8)
    canv.setFillColor(NAVY)
    canv.drawString(LEFT, PAGE_H - 12.5 * mm, "K题 · 场景感知循迹小车")
    canv.setFont("YaHei", 7.2)
    canv.setFillColor(GRAY)
    canv.drawRightString(
        PAGE_W - RIGHT,
        PAGE_H - 12.5 * mm,
        "采购清单 · 2026-10-18",
    )
    canv.setStrokeColor(LINE)
    canv.setLineWidth(0.45)
    canv.line(LEFT, PAGE_H - 15.5 * mm, PAGE_W - RIGHT, PAGE_H - 15.5 * mm)
    canv.restoreState()


def on_first_page(canv, doc):
    pass


def top_view_drawing():
    w = CONTENT_W
    h = 82 * mm
    d = Drawing(w, h)
    d.add(
        Rect(
            0,
            0,
            w,
            h,
            rx=4 * mm,
            ry=4 * mm,
            fillColor=colors.HexColor("#F7F9FB"),
            strokeColor=LINE,
            strokeWidth=0.55,
        )
    )

    # Car body: front faces right.
    bx, by, bw, bh = 43 * mm, 12 * mm, 88 * mm, 58 * mm
    d.add(
        Rect(
            bx,
            by,
            bw,
            bh,
            rx=3 * mm,
            ry=3 * mm,
            fillColor=WHITE,
            strokeColor=NAVY,
            strokeWidth=1.15,
        )
    )

    def label(x, y, text, anchor="start", color=NAVY, size=7.0):
        d.add(
            String(
                x,
                y,
                text,
                fontName="YaHei",
                fontSize=size,
                fillColor=color,
                textAnchor=anchor,
            )
        )

    # Direction
    d.add(Line(136 * mm, 41 * mm, 158 * mm, 41 * mm, strokeColor=GRAY, strokeWidth=0.8))
    d.add(
        Polygon(
            [158 * mm, 41 * mm, 153 * mm, 43 * mm, 153 * mm, 39 * mm],
            fillColor=GRAY,
            strokeColor=GRAY,
        )
    )
    label(158 * mm, 44 * mm, "前进", "end", GRAY, 6.8)

    # Front sensors
    d.add(
        Rect(
            130.5 * mm,
            23 * mm,
            2.4 * mm,
            36 * mm,
            fillColor=ORANGE,
            strokeColor=ORANGE,
        )
    )
    label(135 * mm, 58 * mm, "16 路灰度", "start", NAVY, 6.8)
    d.add(Line(133 * mm, 56 * mm, 135 * mm, 58 * mm, strokeColor=ORANGE, strokeWidth=0.6))

    for yy in (29 * mm, 53 * mm):
        d.add(Circle(134 * mm, yy, 2.1 * mm, fillColor=BLUE, strokeColor=BLUE))
    label(137 * mm, 24 * mm, "2×ToF", "start", BLUE, 6.8)
    d.add(Line(135.5 * mm, 28 * mm, 137 * mm, 25.5 * mm, strokeColor=BLUE, strokeWidth=0.6))

    d.add(Circle(134 * mm, 41 * mm, 2.1 * mm, fillColor=TEAL, strokeColor=TEAL))
    label(137 * mm, 39 * mm, "超声波", "start", TEAL, 6.8)
    d.add(Line(135.5 * mm, 41 * mm, 137 * mm, 40 * mm, strokeColor=TEAL, strokeWidth=0.6))

    # Central camera and IMU
    d.add(Circle(99 * mm, 41 * mm, 4.4 * mm, fillColor=NAVY, strokeColor=NAVY))
    d.add(Circle(99 * mm, 41 * mm, 1.3 * mm, fillColor=WHITE, strokeColor=WHITE))
    label(99 * mm, 47 * mm, "相机（下视）", "middle", NAVY, 6.8)
    d.add(
        Rect(
            107 * mm,
            31 * mm,
            5 * mm,
            5 * mm,
            fillColor=GREEN,
            strokeColor=GREEN,
        )
    )
    label(109.5 * mm, 28.6 * mm, "IMU", "middle", GREEN, 6.4)

    # Battery
    d.add(
        Rect(
            52 * mm,
            23 * mm,
            22 * mm,
            13 * mm,
            fillColor=LIGHT_BLUE,
            strokeColor=BLUE,
            strokeWidth=0.7,
        )
    )
    label(63 * mm, 29 * mm, "3S 电池", "middle", NAVY, 6.8)

    # Rear interface
    d.add(
        Rect(
            45 * mm,
            47 * mm,
            10 * mm,
            13 * mm,
            fillColor=WHITE,
            strokeColor=NAVY,
            strokeWidth=0.7,
        )
    )
    label(50 * mm, 52.5 * mm, "TFT", "middle", NAVY, 6.4)
    d.add(Circle(60 * mm, 65 * mm, 1.8 * mm, fillColor=RED, strokeColor=RED))
    label(63 * mm, 63.8 * mm, "启动", "start", RED, 6.4)

    # Rear line sensor
    d.add(
        Rect(
            40.5 * mm,
            27 * mm,
            2.4 * mm,
            30 * mm,
            fillColor=ORANGE,
            strokeColor=ORANGE,
        )
    )
    label(28 * mm, 41 * mm, "8 路灰度", "middle", NAVY, 6.8)
    d.add(Line(31 * mm, 41 * mm, 40.5 * mm, 41 * mm, strokeColor=ORANGE, strokeWidth=0.6))

    # Wheels
    for yy in (8 * mm, 70 * mm):
        d.add(
            Rect(
                76 * mm,
                yy,
                16 * mm,
                4 * mm,
                fillColor=colors.HexColor("#3B4652"),
                strokeColor=colors.HexColor("#3B4652"),
            )
        )
    label(84 * mm, 76.5 * mm, "驱动轮 ×2", "middle", GRAY, 6.6)

    for xx in (51 * mm, 123 * mm):
        d.add(
            Circle(
                xx,
                41 * mm,
                3 * mm,
                fillColor=GRAY,
                strokeColor=WHITE,
                strokeWidth=0.5,
            )
        )
    label(51 * mm, 35 * mm, "2×牛眼轮", "middle", GRAY, 6.6)

    label(18 * mm, 41 * mm, "后部", "middle", GRAY, 7.0)
    label(160 * mm, 41 * mm, "前部", "middle", GRAY, 6.8)
    label(
        w / 2,
        2.8 * mm,
        "俯视图，不按严格比例；前部朝右。",
        "middle",
        GRAY,
        6.4,
    )
    return d


def build():
    doc = BaseDocTemplate(
        OUTPUT_PDF,
        pagesize=A4,
        leftMargin=LEFT,
        rightMargin=RIGHT,
        topMargin=TOP,
        bottomMargin=BOTTOM,
        title="K题 场景感知循迹小车采购清单",
        author="Codex",
        subject="2026 校电赛低年级组 K 题采购清单与选型建议",
    )
    frame = Frame(
        LEFT,
        BOTTOM,
        CONTENT_W,
        PAGE_H - TOP - BOTTOM,
        id="normal",
        leftPadding=0,
        rightPadding=0,
        topPadding=0,
        bottomPadding=0,
    )
    doc.addPageTemplates(
        [
            PageTemplate(
                id="first",
                frames=[frame],
                onPage=on_first_page,
            ),
            PageTemplate(
                id="later",
                frames=[frame],
                onPage=on_later_pages,
            ),
        ]
    )

    story = []

    # Title block
    title_data = [
        [p("2026 校电赛 · 低年级组 K 题", title_small)],
        [p("场景感知循迹小车", title_main)],
        [p("采购清单与选型建议", title_sub)],
        [
            p(
                "2WD 差速  |  编码器里程  |  16 路灰度  |  ToF + 超声波  |  下视相机",
                title_meta,
            )
        ],
        [p("版本 1.0  ·  2026-10-18  ·  参考价按常见模块估算", title_meta)],
    ]
    title_table = Table(title_data, colWidths=[CONTENT_W])
    title_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), NAVY),
                ("LEFTPADDING", (0, 0), (-1, -1), 14),
                ("RIGHTPADDING", (0, 0), (-1, -1), 14),
                ("TOPPADDING", (0, 0), (-1, -1), 1.8),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 1.8),
                ("TOPPADDING", (0, 0), (-1, 0), 12),
                ("BOTTOMPADDING", (0, -1), (-1, -1), 12),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
            ]
        )
    )
    story.append(title_table)
    story.append(Spacer(1, 2.2 * mm))
    story.append(HRFlowable(width=CONTENT_W, thickness=2.4, color=TEAL))
    story.append(Spacer(1, 4.0 * mm))

    # Summary cards
    summary_data = [
        [
            p("完整版预算", summary_label),
            p("基本版预算", summary_label),
            p("车体上限", summary_label),
            p("建议车体", summary_label),
        ],
        [
            p("¥1,600-2,200", summary_value),
            p("¥900-1,200", summary_value),
            p("35×25 cm", summary_value),
            p("30×20 cm", summary_value),
        ],
    ]
    summary_table = Table(
        summary_data,
        colWidths=[CONTENT_W / 4.0] * 4,
    )
    summary_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), LIGHT_BLUE),
                ("BOX", (0, 0), (-1, -1), 0.55, colors.HexColor("#C9DCEB")),
                ("INNERGRID", (0, 0), (-1, -1), 0.4, WHITE),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("TOPPADDING", (0, 0), (-1, -1), 4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
            ]
        )
    )
    story.append(summary_table)
    story.append(
        p(
            "完整版含 OpenMV、后 8 路、BNO085、第二块电池、场地与备件；不含万用表、示波器、3D 打印等工具。",
            note,
        )
    )

    # One-line plan
    story.extend(section_header("01", "一句话方案"))
    story.append(
        p(
            "这题 60 分在避障和两种自动泊车。建议用 <b>2WD 差速底盘</b>，前 <b>16 路模拟灰度</b>负责循迹，"
            "后 <b>8 路</b>负责倒车和回线，<b>编码器 + IMU</b> 做里程和航向，<b>2 个 ToF + 1 个超声波</b>"
            "负责 8 cm 停车，<b>中央下视广角相机</b>识别车位三边线。普通 8 路数字灰度加 TT 电机的循迹车套件不够用。",
            body,
        )
    )
    story.append(Spacer(1, 2.4 * mm))
    story.append(
        p(
            "<b>关键尺寸：</b>车体投影 ≤35×25 cm；垂直车位开口 29.7 cm，车宽 22 cm 时每边约 3.8 cm 余量；"
            "平行车位深 29.7 cm，车长 30-32 cm 比 35 cm 稳。单圈约 478.5 cm，两圈约 957 cm。",
            body_small,
        )
    )
    story.append(Spacer(1, 1.2 * mm))
    story.append(
        p(
            "<b>方向：</b>顺时针跑时，所有车位都在车体右侧；相机 ROI 可偏右，算法不用做左右镜像。",
            body_small,
        )
    )

    # Chassis
    story.extend(section_header("02", "车架：买裸底盘，不做全 DIY"))
    story.append(
        p(
            "电子部分按模块采购，机械部分可以直接买裸底盘。规则禁止的是能完成任务的成品系统，"
            "不是车架、电机、轮子这些零件。买裸底盘后自己配电控、传感器和支架，仍然属于自主设计、组装与调试。",
            body_small,
        )
    )
    story.append(Spacer(1, 2.0 * mm))

    chassis_rows = [
        [
            p("方案", table_header),
            p("搜索词", table_header),
            p("参考价", table_header),
            p("适合度", table_header),
            p("说明", table_header),
        ],
        [
            p("37 电机 2WD 双层亚克力", table_item),
            p("37电机 两驱 底盘 双层 亚克力", table_cell),
            p("¥60-120", table_cell_center),
            p("首选", ParagraphStyle("g", parent=table_cell_center, textColor=GREEN, fontName="YaHeiB")),
            p("最省事；确认 37-520 孔位、65 mm 轮、牛眼轮。", table_cell),
        ],
        [
            p("2WD 铝合金双层", table_item),
            p("铝合金 双层 小车底盘 2WD 37电机", table_cell),
            p("¥100-200", table_cell_center),
            p("稳", ParagraphStyle("b", parent=table_cell_center, textColor=BLUE, fontName="YaHeiB")),
            p("更硬更重；外形控制在 32×22 cm 内。", table_cell),
        ],
        [
            p("激光切割定制", table_item),
            p("亚克力 激光切割 小车底盘 定制", table_cell),
            p("¥80-180", table_cell_center),
            p("最贴合", ParagraphStyle("t", parent=table_cell_center, textColor=TEAL, fontName="YaHeiB")),
            p("画 DXF；留电机孔、传感器孔、相机柱。", table_cell),
        ],
        [
            p("3D 打印", table_item),
            p("3D打印 小车底盘 PETG", table_cell),
            p("¥50-100", table_cell_center),
            p("打样", table_cell_center),
            p("打样可以；最终承重和刚性不如铝/亚克力。", table_cell),
        ],
        [
            p("TT 电机小底盘", table_item),
            p("TT电机 两驱 底盘 套件", table_cell),
            p("¥20-50", table_cell_center),
            p("不推荐", priority_no),
            p("扭矩和编码器弱，只适合先搭原型。", table_cell),
        ],
        [
            p("4WD / 麦轮 / 履带", table_item),
            p("-", table_cell_center),
            p("-", table_cell_center),
            p("不建议/禁用", priority_no),
            p("滑移转向里程不准；麦轮和履带规则禁止。", table_cell),
        ],
    ]
    chassis_table = Table(
        chassis_rows,
        colWidths=[31 * mm, 50 * mm, 21 * mm, 17 * mm, 59 * mm],
        repeatRows=1,
    )
    chassis_style = [
        ("BACKGROUND", (0, 0), (-1, 0), NAVY),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ALIGN", (2, 1), (3, -1), "CENTER"),
        ("LEFTPADDING", (0, 0), (-1, -1), 2.4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 2.4),
        ("TOPPADDING", (0, 0), (-1, -1), 2.2),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 2.0),
        ("LINEBELOW", (0, 0), (-1, -1), 0.35, LINE),
        ("BOX", (0, 0), (-1, -1), 0.45, LINE),
    ]
    for i in range(1, len(chassis_rows)):
        if i % 2 == 0:
            chassis_style.append(("BACKGROUND", (0, i), (-1, i), PALE))
    chassis_table.setStyle(TableStyle(chassis_style))
    story.append(chassis_table)
    story.append(Spacer(1, 2.4 * mm))
    story.append(
        callout(
            "<b>推荐：</b>37 电机 2WD 双层亚克力底盘 + 牛眼轮 + 3D 打印相机/传感器支架，机械部分约 ¥120-200。"
            "不要买带主板、循迹传感器和完整程序的“智能小车套件”。",
            bg=colors.HexColor("#EAF6F2"),
            border=TEAL,
        )
    )
    story.append(Spacer(1, 3.0 * mm))

    # Chassis checklist
    checklist = [
        [
            p("□ 外形 ≤32×22 cm，最终 ≤35×25 cm", body_small),
            p("□ 双层板，铜柱 40-50 mm", body_small),
        ],
        [
            p("□ 37 mm 电机孔位，匹配 37-520", body_small),
            p("□ 能放 3S 18650 电池组", body_small),
        ],
        [
            p("□ 65 mm 轮，离地 15-25 mm", body_small),
            p("□ 前部能装灰度条、ToF、超声波", body_small),
        ],
        [
            p("□ 2 驱动轮 + 2 牛眼轮", body_small),
            p("□ 卖家能给孔位图/DXF/STEP", body_small),
        ],
    ]
    checklist_table = Table(checklist, colWidths=[CONTENT_W / 2.0] * 2)
    checklist_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), PALE),
                ("BOX", (0, 0), (-1, -1), 0.45, LINE),
                ("INNERGRID", (0, 0), (-1, -1), 0.35, WHITE),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 6),
                ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                ("TOPPADDING", (0, 0), (-1, -1), 3.6),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 3.6),
            ]
        )
    )
    story.append(
        KeepTogether(
            [
                p("车架检查表（买前逐项确认）", subsection),
                Spacer(1, 1.2 * mm),
                checklist_table,
            ]
        )
    )

    # Procurement
    story.append(PageBreak())
    story.extend(section_header("03", "采购清单"))
    story.append(
        p(
            '<font color="#B03A2E"><b>必买</b></font> = 没它无法完成基本要求；'
            '<font color="#2E6E9E"><b>推荐</b></font> = 明显提高完整度或可靠性；'
            '<font color="#7B8794"><b>可选</b></font> = 有余量再买。参考价为单件/单套常见区间，不含运费。',
            note,
        )
    )
    story.append(Spacer(1, 2.0 * mm))

    group_a = [
        ("A1", "编码器减速电机", "JGB37-520，12V 178 rpm，霍尔编码器，6 mm D 轴。不要普通 TT 电机；建议留 1 个备用。", "2", "¥60-80/个", "必买"),
        ("A2", "车轮/牛眼轮", "65 mm 橡胶轮 + 6 mm D 轴轮毂；20 mm 牛眼轮。2WD + 2 牛眼轮 = 4 轮。", "2+2", "¥40-60/套", "必买"),
        ("A3", "电机支架/联轴器", "37 mm 电机支架、6 mm 联轴器；与电机轴径匹配。", "1 套", "¥20-30", "必买"),
        ("A4", "裸底盘", "37 电机 2WD 双层亚克力，30×20 cm 左右；或定制激光切割。不要买带主板/程序的成品车。", "1", "¥60-120", "必买"),
        ("A5", "相机立柱/传感器支架", "3D 打印或激光切割；相机高度 20-25 cm；灰度条高度可调。立柱要刚性。", "1 套", "¥30-60", "推荐"),
        ("A6", "保险杠/理线", "EVA 泡棉、扎带、魔术贴；保险杠计入 35 cm 车长。", "1 套", "¥20", "推荐"),
        ("A7", "铜柱/螺丝/防松螺母", "M3 铜柱 40-50 mm、螺丝套装；双层板结构。", "1 套", "¥20-30", "必买"),
    ]
    story.append(group_header("A. 运动与结构", "车架 / 电机 / 轮系 / 支架"))
    story.append(make_bom_table(group_a))
    story.append(Spacer(1, 3.0 * mm))

    group_b = [
        ("B1", "主控/下载器", "STM32F407VET6 核心板 + ST-Link V2。F411 也够；F407 引脚更宽裕。", "1 套", "¥100-150", "必买"),
        ("B2", "电机驱动", "DRV8871 模块 ×2，或 TB6612FNG ×1 + 散热片；37 电机建议 DRV8871，限流约 2 A。", "1 套", "¥20-50", "必买"),
        ("B3", "主电池", "3S 18650 3000 mAh，15 A 保护板；一块比赛，一块备用。", "2", "¥90-130/组", "必买"),
        ("B4", "充电器", "12.6 V 1 A；不要充错电压。", "1", "¥30-40", "必买"),
        ("B5", "5 V 降压", "3 A，MP1584/LM2596；逻辑和相机独立供电。", "1+1 备", "¥10-20/个", "必买"),
        ("B6", "保险丝/开关/XT30", "5-7.5 A 保险丝、10 A 开关、XT30 插头；电池入口必须保险。", "1 套", "¥30-50", "必买"),
        ("B7", "电源分配/洞洞板", "5×7 cm 或定制 PCB；减少接线错误。", "1", "¥20", "推荐"),
        ("B8", "电容/磁珠/热缩管", "1000 µF/25 V、100 nF、磁珠；电机噪声抑制别省。", "1 套", "¥20-30", "必买"),
        ("B9", "编码器电平转换", "74HC14 或电阻分压；编码器 5 V 输出不能直连 3.3 V。", "1 套", "¥5-10", "推荐"),
    ]
    story.append(group_header("B. 电控与电源", "主控 / 驱动 / 电池 / 抗干扰"))
    story.append(make_bom_table(group_b))
    story.append(Spacer(1, 3.2 * mm))
    story.append(
        callout(
            "<b>到货先验收：</b>① 两个电机正反转，编码器 A/B 都有计数；② 两个 VL53L1X 能分别改地址；"
            "③ HC-SR04 回波经分压后不超过 3.3 V；④ OpenMV 在 20-25 cm 高度能看全 42 cm 车位。",
            bg=colors.HexColor("#F2F5F8"),
            border=colors.HexColor("#AFC0CF"),
        )
    )
    story.append(Spacer(1, 2.4 * mm))
    story.append(
        p(
            "<b>采购备注：</b>先确认底盘孔位、轮径和电机轴径，再买相机和支架；JST/PH2.0 连接器尽量同一家店买齐；"
            "保留型号页和发票，报告和备件都要用。",
            note,
        )
    )

    # Page 3
    story.append(PageBreak())
    group_c = [
        ("C1", "前灰度阵列", "16 路模拟量，8-10 mm 间距，3.3 V 安全。1.8 cm 线宽 + R60；8 路数字不够。", "1+1 备", "¥60-100", "必买"),
        ("C2", "后灰度阵列", "8 路模拟量；倒车入库、回主线。", "1", "¥30-50", "推荐"),
        ("C3", "IMU", "BNO085（推荐）或 MPU6050（预算）；航向，BNO085 更稳。", "1", "¥150-200 / ¥10-20", "推荐"),
        ("C4", "避障 ToF", "VL53L1X，带 XSHUT，2 个；两个同地址，用 XSHUT 改地址或 TCA9548A。", "2", "¥30-40/个", "必买"),
        ("C5", "超声波", "HC-SR04；透明瓶更可靠；回波 5 V 要分压。", "1", "¥5-10", "必买"),
        ("C6", "车位相机", "OpenMV H7 Plus + 广角镜头 + 16 GB microSD；停车 40 分关键，也可用 K230。", "1 套", "¥370-520", "推荐"),
        ("C7", "显示", "2.4 寸 SPI TFT（ST7789/ILI9341）；显示 5 项数据 + 状态。", "1", "¥40-60", "必买"),
        ("C8", "蜂鸣器/按键/LED", "5 V 无源蜂鸣器 + 三极管、启动键、急停、LED；不同任务用不同提示节奏。", "1 套", "¥20-30", "必买"),
        ("C9", "相机补光", "可调白光 LED + 柔光片；现场光不稳时再用。", "1 套", "¥15-25", "可选"),
    ]
    story.append(group_header("C. 感知与交互", "循迹 / 避障 / 车位 / 人机界面"))
    story.append(make_bom_table(group_c))
    story.append(Spacer(1, 3.0 * mm))

    group_d = [
        ("D1", "场地板", "180×160 cm 白色哑光 PVC 发泡板/地垫；题目要求场地自备。", "1", "¥100-200", "必买"),
        ("D2", "标线胶带", "18 mm 黑色哑光电工胶带；1.8 cm 正好，哑光防反光。", "3-4 卷", "¥10-15/卷", "必买"),
        ("D3", "车位/障碍", "A4 纸 20 张、500 mL 塑料瓶、卷尺/直尺/量角器；两张 A4 拼成 42×29.7。", "1 套", "¥50-80", "必买"),
        ("D4", "连接器/线材", "杜邦线、JST-XH、PH2.0、18/22/26 AWG；电机线不要用细杜邦。", "1 套", "¥50-80", "必买"),
        ("D5", "备件", "TCRT5000、保险丝、降压、驱动、螺丝；现场最怕小件坏。", "1 套", "¥50-100", "推荐"),
        ("D6", "工具（借用）", "万用表、12 V 5 A 电源、示波器、烙铁、3D 打印/激光切割；先问实验室。", "1 套", "实验室", "可选"),
    ]
    story.append(group_header("D. 场地、工具与备件", "自备场地 / 耗材 / 易损件"))
    story.append(make_bom_table(group_d))

    # Budget summary
    story.append(Spacer(1, 4.0 * mm))
    story.append(p("预算汇总", subsection))
    story.append(Spacer(1, 1.2 * mm))
    budget_rows = [
        [p("配置", table_header), p("包含", table_header), p("参考预算", table_header)],
        [
            p("基本版", table_item),
            p("2WD + 16 路灰度 + MPU6050 + ToF/超声波 + TFT + 1 块电池 + 场地；不含相机、后 8 路、备件。", table_cell),
            p("¥900-1,200", table_cell_center),
        ],
        [
            p("完整版", table_item),
            p("基本版 + OpenMV + 后 8 路 + BNO085 + 第二块电池 + 备件 + 更好车架；不含工具。", table_cell),
            p("¥1,600-2,200", table_cell_center),
        ],
        [
            p("工具", table_item),
            p("万用表、12 V 5 A 电源、示波器、烙铁、3D 打印/激光切割。", table_cell),
            p("借用", table_cell_center),
        ],
    ]
    budget_table = Table(budget_rows, colWidths=[24 * mm, 122 * mm, 32 * mm])
    budget_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), NAVY),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("ALIGN", (2, 1), (2, -1), "CENTER"),
                ("LEFTPADDING", (0, 0), (-1, -1), 3.0),
                ("RIGHTPADDING", (0, 0), (-1, -1), 3.0),
                ("TOPPADDING", (0, 0), (-1, -1), 2.4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 2.2),
                ("LINEBELOW", (0, 0), (-1, -1), 0.35, LINE),
                ("BOX", (0, 0), (-1, -1), 0.45, LINE),
                ("BACKGROUND", (0, 1), (-1, 1), PALE),
            ]
        )
    )
    story.append(budget_table)
    story.append(
        p(
            "价格按 2026 年 10 月常见模块估算；OpenMV、BNO085、第二块电池和场地是预算波动最大的四项。",
            note,
        )
    )
    story.append(Spacer(1, 4.0 * mm))
    story.append(p("车体传感器布局建议", subsection))
    story.append(Spacer(1, 1.2 * mm))
    layout_rows = [
        [p("位置", table_header), p("装什么", table_header), p("注意", table_header)],
        [
            p("前部", table_item),
            p("16 路灰度、2×VL53L1X、HC-SR04", table_cell),
            p("灰度离地 3-6 mm；ToF/超声波 10-12 cm 高。", table_cell),
        ],
        [
            p("后部", table_item),
            p("8 路灰度、TFT、启动键", table_cell),
            p("倒车入库和回线用；屏幕朝评委。", table_cell),
        ],
        [
            p("中央", table_item),
            p("相机立柱、IMU", table_cell),
            p("相机 20-25 cm 高、广角；IMU 远离电机。", table_cell),
        ],
        [
            p("底部", table_item),
            p("3S 电池、驱动轮、2 个牛眼轮", table_cell),
            p("电池低且居中；牛眼轮选低摩擦。", table_cell),
        ],
    ]
    layout_table = Table(layout_rows, colWidths=[18 * mm, 76 * mm, 84 * mm])
    layout_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), NAVY),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 3.0),
                ("RIGHTPADDING", (0, 0), (-1, -1), 3.0),
                ("TOPPADDING", (0, 0), (-1, -1), 2.4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 2.2),
                ("LINEBELOW", (0, 0), (-1, -1), 0.35, LINE),
                ("BOX", (0, 0), (-1, -1), 0.45, LINE),
                ("BACKGROUND", (0, 1), (-1, 1), PALE),
                ("BACKGROUND", (0, 3), (-1, 3), PALE),
            ]
        )
    )
    story.append(layout_table)
    story.append(Spacer(1, 4.0 * mm))
    story.append(p("关键计算和调试顺序", subsection))
    story.append(Spacer(1, 1.2 * mm))
    story.append(
        p(
            "<b>电机：</b>37-520 178 rpm + 65 mm 轮，理论约 0.6 m/s；编码器约 0.15 mm/计数。"
            "<b>灰度：</b>16 路按 8 mm 间距约覆盖 12.8 cm，加权平均分辨率约 4 mm。"
            "<b>相机：</b>25 cm 高 + 90° 镜头约看 50 cm 宽，42 cm 车位能看全。",
            body_small,
        )
    )
    story.append(Spacer(1, 1.5 * mm))
    story.append(
        p(
            "<b>调试顺序：</b>直线 -> R60 弯 -> 两圈停 A -> 8 cm 避障（透明/不透明瓶）-> 垂直入库 -> 侧方停车 -> 组合跑。"
            "每项至少重复 10 次，再上比赛场地。B/C 点不能贴标记，用陀螺仪角速度或累计角度触发蜂鸣。",
            body_small,
        )
    )

    # Page 4
    story.append(PageBreak())
    story.extend(section_header("04", "采购顺序与验收"))

    sequence_rows = [
        [p("批次", table_header), p("买什么", table_header), p("目的", table_header)],
        [
            p("第一批", table_item),
            p("车架、电机、轮、驱动、主控、电源、16 路灰度、屏幕、ToF/超声波。", table_cell),
            p("先把直线、R60 弯、两圈停 A 和 8 cm 避障跑稳。", table_cell),
        ],
        [
            p("第二批", table_item),
            p("OpenMV、后 8 路、BNO085、相机/传感器支架。", table_cell),
            p("做车位识别、垂直入库、侧方停车。", table_cell),
        ],
        [
            p("第三批", table_item),
            p("场地、第二块电池、备件、补光。", table_cell),
            p("完整跑、重复测试、比赛备份。", table_cell),
        ],
    ]
    sequence_table = Table(sequence_rows, colWidths=[20 * mm, 92 * mm, 66 * mm])
    sequence_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), NAVY),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 3.0),
                ("RIGHTPADDING", (0, 0), (-1, -1), 3.0),
                ("TOPPADDING", (0, 0), (-1, -1), 2.6),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 2.4),
                ("LINEBELOW", (0, 0), (-1, -1), 0.35, LINE),
                ("BOX", (0, 0), (-1, -1), 0.45, LINE),
                ("BACKGROUND", (0, 1), (-1, 1), PALE),
            ]
        )
    )
    story.append(sequence_table)
    story.append(Spacer(1, 3.5 * mm))

    story.append(p("验收清单（装好后逐项打勾）", subsection))
    story.append(Spacer(1, 1.2 * mm))
    accept_rows = [
        [
            p("□ 车体投影 ≤35×25 cm", body_small),
            p("□ 4 轮：2 驱动 + 2 牛眼轮；无麦轮/履带", body_small),
        ],
        [
            p("□ 16 路灰度离地 3-6 mm，有遮光罩", body_small),
            p("□ 2 个 ToF + 超声波在 10-12 cm 高", body_small),
        ],
        [
            p("□ 相机 20-25 cm 高，广角能看到 42 cm 车位", body_small),
            p("□ TFT 显示两圈耗时 + 3 类计数", body_small),
        ],
        [
            p("□ 蜂鸣器能区分 B/C、避障、离轨", body_small),
            p("□ 启动键 + 急停；无遥控", body_small),
        ],
        [
            p("□ 电池保险丝、开关、独立 5 V", body_small),
            p("□ 两圈停在 A；障碍 5-7 cm 停", body_small),
        ],
        [
            p("□ 垂直入库不压线", body_small),
            p("□ 侧方停车不压线", body_small),
        ],
    ]
    accept_table = Table(accept_rows, colWidths=[CONTENT_W / 2.0] * 2)
    accept_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), PALE),
                ("BOX", (0, 0), (-1, -1), 0.45, LINE),
                ("INNERGRID", (0, 0), (-1, -1), 0.35, WHITE),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 6),
                ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                ("TOPPADDING", (0, 0), (-1, -1), 3.4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 3.4),
            ]
        )
    )
    story.append(accept_table)
    story.append(Spacer(1, 3.5 * mm))

    story.append(Spacer(1, 3.0 * mm))
    story.append(
        callout(
            "<b>最容易翻车的三点：</b>相机标准镜头看不全 42 cm 车位；透明塑料瓶 ToF 漏检；"
            "灰度把车位边线当成岔路。三个问题都要在第一批到货后专门搭台测试。",
            bg=colors.HexColor("#FFF4E5"),
            border=ORANGE,
        )
    )
    story.append(Spacer(1, 3.5 * mm))
    story.append(p("车体布局建议（俯视）", subsection))
    story.append(Spacer(1, 1.2 * mm))
    story.append(top_view_drawing())
    story.append(
        p(
            "前部集中放灰度、ToF、超声波；中央放相机和 IMU；后部放 TFT、按键；电池低且居中。",
            note,
        )
    )
    story.append(Spacer(1, 3.0 * mm))
    story.append(
        callout(
            "<b>规则提醒：</b>禁止成品系统；裸底盘、电机、轮子、结构件一般可用，但建议先问组委会。"
            "3-4 轮，不得履带或麦轮；车体 ≤35×25 cm；车载电池；行驶中不得遥控或人工干预。"
            "场地除黑线外不得有 A/B/C/D 或内部区域标识。",
            bg=colors.HexColor("#F2F5F8"),
            border=colors.HexColor("#AFC0CF"),
        )
    )
    story.append(Spacer(1, 3.0 * mm))
    story.append(
        p(
            "依据：2026 年电子电赛试题(1).docx，低年级组 K 题。本清单供队内采购和备赛使用，价格与库存以实际店铺为准。",
            note,
        )
    )

    doc.build(story, canvasmaker=NumberedCanvas)


if __name__ == "__main__":
    build()
