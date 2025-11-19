'''
server_code.Points 的 Docstring

- 用于解析svg文件中的路径数据，并将其转换为点集；
- 转换为JSON文件直接发包到server。
'''

# 导入必要的库
import numpy as np
import os
import math
from svgpathtools import parse_path # svg解析库
import json
import xml.etree.ElementTree as ET # xml文件树结构化
import Const as CSN # 常量文件

def read_viewbox(svg_path) -> tuple:
    '''
    读取svg文件的viewbox信息，也就是svg的坐标系范围。坐标系原点在左上角，x轴向右，y轴向下。
    0.0,0.0,w,h代表左上角坐标和宽高。
    '''
    root = ET.parse(svg_path).getroot()
    vb = root.get('viewBox') or root.get('viewbox')
    if vb:
        vals = list(map(float, vb.strip().split()))
        return tuple(vals)  # x,y,w,h
    w = float(root.get('width', '1024'))
    h = float(root.get('height', '1024'))
    return (0.0, 0.0, w, h)

def split_path_into_subpaths(path_obj, tol=1e-6) -> list:
    """把 svgpathtools.Path 拆成连续的子 Path 列表，如果有多个path"""
    segs = list(path_obj)
    if not segs:
        return []
    subs = []
    cur = [segs[0]]
    prev_end = segs[0].end
    for s in segs[1:]:
        if abs(s.start.real - prev_end.real) > tol or abs(s.start.imag - prev_end.imag) > tol:
            subs.append(parse_path(''.join([seg.d() for seg in cur])) if hasattr(cur[0], 'd') else type(path_obj)(*cur))
            cur = [s]
        else:
            cur.append(s)
        prev_end = s.end
    if cur:
        subs.append(type(path_obj)(*cur))
    return subs # 返回的是 svgpathtools.Path 对象列表

def sample_path_by_arclen(path_obj, step=15.0) -> list:
    """
    按弧长均匀采样 svgpathtools.Path，返回点列表 [[x1,y1], [x2,y2], ...]
    对每个Path来进行单独采样
    - 参数列表
    path_obj: svgpathtools.Path 对象
    step: 采样步长，单位与svg坐标系一致，默认15.0，即每15个单位采样一个点
    """
    L = path_obj.length(error=1e-5) # L是路径总长度
    if L <= 1e-9:
        p = path_obj.point(0)
        return [[float(p.real), float(p.imag)]]
    n = max(2, int(math.ceil(L/step)) + 1) # n是采样点数
    dense_n = max(500, 5 * n) # 总共采样点数为 dense_n
    ts = np.linspace(0.0, 1.0, dense_n)
    pts = np.array([path_obj.point(t) for t in ts], dtype=complex)
    xs = pts.real
    ys = pts.imag
    segd = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    cum = np.concatenate(([0.0], np.cumsum(segd)))
    samp_d = np.linspace(0.0, cum[-1], n)
    xs_s = np.interp(samp_d, cum, xs)
    ys_s = np.interp(samp_d, cum, ys)
    return [[float(x), float(y)] for x, y in zip(xs_s, ys_s)]

def process_svg(svg_path, id, sample_step=8.0, out_json=None) -> str:
    """
    process_svg 的 Docstring
    
    解析 svg 文件，提取路径并采样为点集，保存为 JSON 文件。
    """
    if out_json is None:
        out_json = "./JSON/"+str(id)+"_points.json"
    tree = ET.parse(svg_path)
    root = tree.getroot()
    vb = read_viewbox(svg_path)
    vb_x, vb_y, vb_w, vb_h = vb
    cx = vb_x + vb_w / 2.0
    cy = vb_y + vb_h / 2.0

    results = []
    paths = root.findall('.//{http://www.w3.org/2000/svg}path') or root.findall('.//path')
    for i, path_el in enumerate(paths):
        d = path_el.get('d')
        attrs = {k: path_el.get(k) for k in ('id','fill','stroke','stroke-width')}
        if not d:
            continue
        path_obj = parse_path(d)
        subs = split_path_into_subpaths(path_obj)
        if not subs:
            subs = [path_obj]
        for j, sub in enumerate(subs):
            pts = sample_path_by_arclen(sub, step=sample_step)
            # shift origin to svg center->移动到坐标系中心
            # 同时进行缩放
            scale_w_or_h = vb_w if vb_w > vb_h else vb_h
            scale_factor = CSN.RAIDUS * 0.95 / scale_w_or_h
            pts_centered = [[(x - cx) * scale_factor, (y - cy) * scale_factor] for x,y in pts]
            results.append({
                "orig_path_index": i,
                "subpath_index": j,
                "attrs": attrs,
                "points": pts_centered
            })
    with open(out_json, 'w', encoding='utf-8') as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    return out_json
