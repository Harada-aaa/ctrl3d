# ctrl3d

PythonSCAD 上で動く機構設計（メカニズム設計）のための小さな Python モジュールです。
節（リンク）と対偶（ジョイント）で機構を記述し、**自由度の計算・閉ループの組み立て・
PythonSCAD での表示**までを一貫して行えます。

- 入力節・中間節・固定節・出力節を役割として記述
- 回転対偶（R 対偶）と直進対偶（P 対偶）
- 平面／空間の自動判定と **Grübler–Kutzbach の自由度**
- **スクリュ理論による瞬間自由度**（過拘束・特異姿勢の検出）
- ループ閉合誤差の評価と、入力角を与えての**数値的な位置解析**
- 平面四節リンクの **Grashof 判定**（クランクロッカー／両クランク／両ロッカー）
- PythonSCAD での表示（`m.show()`）と OpenSCAD ファイル出力（`m.to_scad()`）

依存パッケージはありません（Python 標準ライブラリのみ）。Python 3.9 以上。

---

## インストールと実行

```powershell
# リポジトリのルートで
py -3 -m pip install -e .

# テスト
py -3 -m unittest discover -s tests -t .
```

PythonSCAD 本体（v1.1.2 で確認）では、スクリプトを開いて実行するだけで
`m.show()` がビューアに表示されます。PythonSCAD の Python は 3.14 です。
`ctrl3d` は PythonSCAD に依存しないので、通常の Python で設計計算だけを行うこともできます。

---

## クイックスタート（平面四節リンク）

```python
from ctrl3d import Mechanism

m = Mechanism("four_bar")

# 固定節（グラウンド）: 2 つのピン間距離 0.40
ground = m.fixed("ground", spacing=0.40)

# 入力節（クランク）、中間節（コネクティングロッド）、出力節（ロッカー）
crank   = m.input("crank", length=0.12)
coupler = m.intermediate("coupler", length=0.40)
rocker  = m.output("rocker", length=0.30)

# 回転対偶で閉ループを構成
m.revolute(ground["A"],  crank["A"],   name="J1")   # 入力対偶（自動的に actuated）
m.revolute(crank["B"],   coupler["A"], name="J2")
m.revolute(coupler["B"], rocker["A"],  name="J3")
m.revolute(rocker["B"],  ground["B"],  name="J4")

print(m.describe())
print("自由度:", m.dof())

# 入力角 45° でループを閉じる（位置解析）
result = m.solve("J1", 45.0)
print(result, "closure =", m.max_closure_error())

m.show()   # PythonSCAD のビューアに表示
```

`m.describe()` の出力例:

```text
ctrl3d mechanism 'four_bar'
  links       : 4 (fixed=1, input=1, intermediate=1, output=1)
  joints      : 4 (revolute=4, prismatic=0)
  loops       : 1
  mobility    : 1 (planar, Gruebler), screw=1, inputs=1, remaining=0
  closure     : max 2.741e-15
  Grashof     : Grashof crank-rocker (shortest link: 'crank')
  diagnostics : ok
```

---

## 記述の考え方

### 節（リンク）

節は剛体で、**名前付きのポート**を持ちます。対偶は 2 つのポートを接続します。

```python
# 長さで指定（ポート A=(0,0,0), B=(length,0,0) が自動生成）
link = m.intermediate("coupler", length=0.40)

# 任意のポート座標で指定（固定節など）
ground = m.fixed("ground", ports={"A": (0, 0, 0), "B": (0.40, 0, 0)})
```

役割クラスは単体でも使えます。

```python
from ctrl3d import FixedLink, InputLink, IntermediateLink, OutputLink

InputLink("crank", length=0.12)
FixedLink("frame", {"A": (0, 0, 0), "B": (0.4, 0, 0)})
```

| 役割 | `LinkRole` | 意味 |
|---|---|---|
| 固定節 | `fixed` | 世界座標に固定された節（グラウンド） |
| 入力節 | `input` | 駆動入力を受ける節 |
| 中間節 | `intermediate` | カプラー、ロッカーなどの伝達節 |
| 出力節 | `output` | 出力を取り出す節 |

### 対偶（ジョイント）

```python
# 回転対偶（R 対偶）: 軸まわりの 1 自由度回転。角度の単位は度。
m.revolute("ground.A", "crank.A", axis=(0, 0, 1), limits=(-90, 90))

# 直進対偶（P 対偶）: 軸方向の 1 自由度並進。値の単位は長さ。
m.prismatic("ground.B", "slider.B", axis=(1, 0, 0), limits=(-0.25, 0.10))
```

- 軸は**リンク A 側のポート座標系**で指定します（省略時は R が +Z、P が +X）。
- エンドポイントは `link["A"]`、`"link.A"`、`(link, "A")`、単一ポートの `link` で指定できます。
- 入力節と固定節を結ぶ対偶は自動的に `actuated=True` になります。手動指定も可能です。
- `joint.value` は R では角度（度）、P では変位（長さ）です。`joint.angle` / `joint.displacement`
  も使えます。

### 自由度

```python
m.dof()                  # 自動判定（平面 or 空間）の Grübler–Kutzbach 自由度
m.dof("planar")          # 平面機構として: F = 3(N - 1) - 2J
m.dof("spatial")         # 空間機構として: F = 6(N - 1) - 5J
m.dof("screw")           # 現在姿勢でのスクリュ理論による瞬間自由度
m.mobility_report()      # 内訳、平面法線、入力数、残り自由度、警告
m.screw_dof()            # スクリュ行列のランクから計算
```

ここで `N` は節の数（固定節を含む）、`J` は 1 自由度対偶の数です。
固定節が複数ある場合は可動節数 `N - (固定節数)` で数えます。
平面／空間の自動判定は、すべての R 対偶軸が平行で、P 対偶軸がその法線に直交する場合に
平面と判定します。

**Grübler とスクリュ自由度の違い**もレポートします。両者が一致しない場合は、
過拘束（Sarrus 機構など）や特異姿勢の可能性があります。スクリュ自由度は
「現在の関節値での瞬間的な値」である点に注意してください。折り畳み姿勢では
Grübler より大きくなることがあります（その場合は警告が出ます）。

入力数は `m.input_count`、残り自由度は `mobility_report().remaining` です。
`remaining == 0` なら、入力数を与えることで姿勢が決まります。

### ループ閉合と位置解析

ツリー（固定節を根とする）で各節の姿勢を決め、ループ対偶の残差を評価します。

```python
m.max_closure_error()          # ループ閉合誤差（長さ + 角度×代表長さ）
m.closure_errors()             # 対偶ごとの ClosureError(position, angle)
m.poses()                      # 各節の 4x4 相当の Transform
m.point("rocker", "B")         # 節のポートの世界座標
```

入力が与えられたときの受動関節の値を求めるには `solve` を使います。

```python
m.solve("J1", 45)                        # 1 自由度機構
m.solve(drivers={"J1": 30, "J2": 50})    # 2 自由度機構（五節リンクなど）
```

減衰付き Gauss–Newton 法で、ループ閉合残差を最小化します。解は初期値に依存するため、
複数解がある機構では初期姿勢（各対偶の `value`）を変えてください。`limits` がある対偶は
範囲内にクランプされます。解けない場合は `SolveResult.converged == False` になります。

### Grashof 判定

平面四節リンク（4 節・4 回転対偶・1 ループ・固定節 1 つ）では、
最短節 `s`、最長節 `l`、残り `p, q` に対して `s + l <= p + q` を判定します。

```python
g = m.grashof()
print(g.kind)        # crank-rocker / double-crank (drag-link) / double-rocker /
                     # triple-rocker / change-point
```

---

## PythonSCAD での使い方

PythonSCAD のスクリプトとして実行すると、`m.show()` がそのままビューアに表示されます。
内部では PythonSCAD の `cube` / `cylinder` / `sphere` / `multmatrix` / `color` / `show` を
呼び出しています（`from pythonscad import *` は不要です）。

```python
from ctrl3d import Mechanism

m = Mechanism("demo")
# ... 節と対偶を定義 ...
m.show()             # PythonSCAD での表示
```

通常の Python で実行した場合、`m.show()` は「PythonSCAD 上で実行してください」という
エラーになります。代わりに OpenSCAD ソースを出力できます。

```python
m.to_scad()                    # str を返す
m.save_scad("mechanism.scad")  # ファイルへ保存
m.export("mechanism.scad")     # 拡張子 .scad なら保存
m.export("mechanism.stl")      # PythonSCAD 上なら STL/3MF なども出力
```

`examples/four_bar.py` などは、PythonSCAD 上では表示、通常の Python では
`.scad` の生成までを自動で行います。

### 表示の見た目

- 固定節: 灰色・半透明・やや太め
- 入力節: 赤、中間節: 灰色、出力節: 青
- 回転対偶: 黒いピン
- 直進対偶: レールとスライダブロック

節の色・半径は `Link(..., color="#ff8800", radius=0.03)` で変更できます。

---

## サンプル

| ファイル | 内容 |
|---|---|
| `examples/four_bar.py` | 平面四節リンク（入力・中間・固定・出力、Grashof、位置解析） |
| `examples/slider_crank.py` | スライダクランク（R 対偶＋P 対偶） |
| `examples/five_bar.py` | 五節リンク（2 自由度、2 入力で閉ループ） |
| `examples/spatial_arm.py` | 空間 3R アーム（空間自由度） |

```powershell
py -3 examples/four_bar.py
```

---

## API の要点

### `Mechanism`

| メソッド・プロパティ | 説明 |
|---|---|
| `fixed/input/intermediate/output(name, ...)` | 役割付きの節を追加 |
| `revolute(a, b, axis=..., value=..., limits=..., actuated=...)` | 回転対偶を追加 |
| `prismatic(a, b, axis=..., value=..., limits=..., actuated=...)` | 直進対偶を追加 |
| `dof(space="auto")` | 自由度（`planar` / `spatial` / `screw` も指定可） |
| `screw_dof()` | スクリュ理論による瞬間自由度 |
| `mobility_report()` | 自由度の内訳と警告 |
| `planarity()` | 平面判定と法線 |
| `poses()` / `point(link, port)` | 現在姿勢 |
| `closure_errors()` / `max_closure_error()` | ループ閉合誤差 |
| `solve(driver, value)` / `solve(drivers={...})` | 位置解析 |
| `grashof()` | 四節リンクの Grashof 判定 |
| `validate()` / `describe()` | 検証とレポート |
| `to_scad()` / `save_scad()` / `show()` / `export()` | 出力・表示 |

### 結果オブジェクト

`MobilityReport`、`GrashofResult`、`ClosureError`、`Diagnostics`、`SolveResult` は
不変のデータクラスで、`str()` で読みやすく表示できます。

---

## 仕組みと設計上の前提

- 節は剛体、対偶は 1 自由度の R または P のみを扱います（歯車・カム・球対偶は未実装）。
- 対偶は 2 節を結ぶバイナリ対偶です。1 つのポートに複数の対偶を付けることは
  できますが、描画は 1 対偶／ポートを前提としています。
- 姿勢計算は固定節を根とする木で行い、ループは独立ループごとの閉合残差で評価します。
- スクリュ自由度は現在の関節値での瞬間値です。特異姿勢では Grübler とずれることがあります。
- 数値解は局所解です。多自由度機構では入力値と初期値を変えて探索してください。

---

## 用語対応

| 日本語 | このモジュールでの名前 |
|---|---|
| 節、リンク | `Link` |
| 入力節 | `InputLink` / `LinkRole.INPUT` |
| 中間節 | `IntermediateLink` / `LinkRole.INTERMEDIATE` |
| 固定節、グラウンド | `FixedLink` / `LinkRole.FIXED` |
| 出力節 | `OutputLink` / `LinkRole.OUTPUT` |
| 回転対偶、R 対偶 | `RevoluteJoint` |
| 直進対偶、P 対偶 | `PrismaticJoint` |
| 自由度 | `Mechanism.dof()` |
| ループ閉合 | `Mechanism.solve()` / `closure_errors()` |
| グラスホフ条件 | `Mechanism.grashof()` |

---

## テスト

```powershell
py -3 -m unittest discover -s tests -t . -v
```

数学（回転・変換・ランク）、節と対偶、平面／空間の自由度、四節・スライダクランク・
五節リンクの位置解析、Grashof 判定、OpenSCAD 出力、PythonSCAD ブリッジ（モック）を
テストしています。
