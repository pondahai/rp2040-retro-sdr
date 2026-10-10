"""分析 M4 錄下的 RECnnn.WAV（寬頻原始樣本，500 ksps）。

    python tools/rec_analyze.py REC000.WAV [--lo 30000 --hi 50000] [--iq 40000]

輸出（放在 .wav 旁邊）：
    RECnnn_report.txt     最強的幾根訊號、每根的強度隨時間怎麼變
    RECnnn_spectrum.png   整段平均頻譜，0–250 kHz 與 lo–hi 放大兩張
    RECnnn_wfall.png      lo–hi 的瀑布圖（時間往下）
    RECnnn_iq.wav         --iq 給了才產生：以該頻率為中心的 I/Q（立體聲，50 ksps），
                          SDR++、SDR# 之類的軟體可以直接開來聽

檔案格式（見 firmware/wav.h 與 RetroSDR.ino 的 M4 段）：
    - 每個樣本是 ADC 原值 0..4095，帶約 500 的直流
    - 每 32768 點一塊，塊開頭 512 點是偏壓回穩的暫態，這裡丟掉
    - 塊與塊之間有鍵盤掃描的空隙；同名 .TXT 有每塊的開始時間（µs）

只用 numpy 與 Pillow。
"""
import argparse
import os
import sys
import wave

import numpy as np
from PIL import Image, ImageDraw


def read_meta(txt):
    meta, starts = {}, []
    if not os.path.exists(txt):
        return meta, starts
    in_list = False
    for line in open(txt, encoding="ascii"):
        line = line.strip()
        if not line:
            continue
        if in_list:
            starts.append(int(line))
        elif line == "start_us":
            in_list = True
        else:
            k, v = line.split(None, 1)
            meta[k] = v
    return meta, starts


def read_blocks(path):
    w = wave.open(path, "rb")
    fs = w.getframerate()
    x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64)
    meta, starts = read_meta(os.path.splitext(path)[0] + ".TXT")
    blk = int(meta.get("block", 32768))
    skip = int(meta.get("skip", 512))
    nb = len(x) // blk
    blocks = x[: nb * blk].reshape(nb, blk)[:, skip:]
    blocks = blocks - blocks.mean(axis=1, keepdims=True)
    return fs, blocks, meta, starts, skip


def segment_spectra(blocks, nfft):
    """每塊切成不重疊的 nfft 段，回傳 (段數, nfft/2+1) 的功率（dBFS，滿刻度正弦 = 0）。"""
    win = np.hanning(nfft)
    gain = win.sum() / 2 * 2048.0           # 振幅 2048 的正弦 -> 0 dBFS
    per = blocks.shape[1] // nfft
    segs = blocks[:, : per * nfft].reshape(-1, nfft) * win
    p = np.abs(np.fft.rfft(segs, axis=1)) / gain
    return 20 * np.log10(p + 1e-12), per


def peaks(db_avg, freqs, lo, hi, n=8, guard=5):
    sel = np.where((freqs >= lo) & (freqs <= hi))[0]
    d = db_avg.copy()
    out = []
    nf = np.median(db_avg[sel])
    for _ in range(n):
        i = sel[np.argmax(d[sel])]
        if d[i] < nf + 6:
            break
        out.append(i)
        d[max(0, i - guard): i + guard + 1] = -999
    return out, nf


def colormap(v):
    """0..1 -> 黑藍青黃紅白（同掌機的瀑布圖）。"""
    stops = np.array([[0, 0, 0], [0, 0, 255], [0, 255, 255], [255, 255, 0], [255, 0, 0], [255, 255, 255]], float)
    t = np.clip(v, 0, 1) * (len(stops) - 1)
    i = np.minimum(t.astype(int), len(stops) - 2)
    f = (t - i)[..., None]
    return (stops[i] * (1 - f) + stops[i + 1] * f).astype(np.uint8)


def plot_spectrum(path, freqs, db, lo, hi, marks):
    W, H, M = 900, 260, 40
    img = Image.new("RGB", (W, 2 * H), "white")
    d = ImageDraw.Draw(img)
    for k, (a, b) in enumerate([(0, freqs[-1]), (lo, hi)]):
        y0 = k * H
        sel = (freqs >= a) & (freqs <= b)
        f, v = freqs[sel], db[sel]
        top = np.ceil(v.max() / 10) * 10
        bot = top - 80
        def X(fr): return M + (fr - a) / (b - a) * (W - 2 * M)
        def Y(dv): return y0 + 20 + (top - dv) / (top - bot) * (H - 50)
        for g in np.arange(bot, top + 1, 10):
            d.line([(M, Y(g)), (W - M, Y(g))], fill=(225, 225, 225))
            d.text((2, Y(g) - 5), f"{g:.0f}", fill="gray")
        for t in np.linspace(a, b, 11):
            d.text((X(t) - 15, y0 + H - 25), f"{t/1000:.1f}k", fill="gray")
        pts = [(X(fr), Y(max(dv, bot))) for fr, dv in zip(f, v)]
        d.line(pts, fill=(0, 90, 160))
        for i in marks:
            if a <= freqs[i] <= b:
                d.text((X(freqs[i]) + 3, Y(db[i]) - 12), f"{freqs[i]/1000:.3f}k", fill="red")
        d.text((M, y0 + 3), f"average spectrum {a/1000:.0f}-{b/1000:.0f} kHz (dBFS)", fill="black")
    img.save(path)


def plot_wfall(path, freqs, dbs, lo, hi, seg_s):
    sel = (freqs >= lo) & (freqs <= hi)
    m = dbs[:, sel]
    nf = np.median(m)
    v = (m - nf + 5) / 45
    img = Image.fromarray(colormap(v))
    w = max(600, img.width)
    img = img.resize((w, max(200, img.height)), Image.NEAREST)
    canvas = Image.new("RGB", (img.width, img.height + 20), "white")
    canvas.paste(img, (0, 20))
    ImageDraw.Draw(canvas).text(
        (4, 4), f"{lo/1000:.1f}-{hi/1000:.1f} kHz, {seg_s*1000:.0f} ms/row, {m.shape[0]} rows", fill="black")
    canvas.save(path)


def write_iq(path, blocks, fs, center, out_fs=50000):
    """以 center 為中心混頻、低通、降到 out_fs，寫成立體聲 16 位元（L=I, R=Q）。"""
    dec = fs // out_fs
    x = blocks.reshape(-1)                  # 塊與塊直接接起來（空隙約 0.7 ms，接縫會有喀聲）
    n = np.arange(len(x))
    bb = x * np.exp(-2j * np.pi * center / fs * n)
    taps = 255
    k = np.arange(taps) - (taps - 1) / 2
    h = np.sinc(2 * 0.4 * out_fs / fs * k) * np.hamming(taps)
    h /= h.sum()
    y = np.convolve(bb, h, mode="same")[::dec]
    y = y / (np.abs(y).max() + 1e-12) * 0.9 * 32767
    iq = np.empty(2 * len(y), dtype="<i2")
    iq[0::2] = np.round(y.real)
    iq[1::2] = np.round(y.imag)
    w = wave.open(path, "wb")
    w.setnchannels(2)
    w.setsampwidth(2)
    w.setframerate(out_fs)
    w.writeframes(iq.tobytes())
    w.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav")
    ap.add_argument("--lo", type=float, default=30000)
    ap.add_argument("--hi", type=float, default=50000)
    ap.add_argument("--nfft", type=int, default=16384, help="每段點數；16384 = 30.5 Hz 解析度、32.8 ms 一段")
    ap.add_argument("--iq", type=float, help="輸出以這個頻率為中心的 I/Q wav")
    a = ap.parse_args()

    fs, blocks, meta, starts, skip = read_blocks(a.wav)
    base = os.path.splitext(a.wav)[0]
    dbs, per = segment_spectra(blocks, a.nfft)
    freqs = np.fft.rfftfreq(a.nfft, 1 / fs)
    avg = 10 * np.log10(np.mean(10 ** (dbs / 10), axis=0))
    marks, nf = peaks(avg, freqs, a.lo, a.hi)
    allmarks, _ = peaks(avg, freqs, 1000, fs / 2, n=12)

    L = []
    L.append(f"file {a.wav}: {blocks.shape[0]} blocks, {blocks.size/fs:.2f} s of samples, fs {fs}")
    if meta:
        L.append("meta: " + ", ".join(f"{k}={v}" for k, v in meta.items()))
    if len(starts) > 1:
        dt = np.diff(starts)
        nominal = (blocks.shape[1] + skip) / fs * 1e6
        L.append(f"block period {dt.mean():.0f} us (min {dt.min()}, max {dt.max()}), "
                 f"sampling {nominal:.0f} us -> gap {dt.mean()-nominal:.0f} us; "
                 f"{int(np.sum(dt > 1.5 * nominal))} blocks dropped")
    L.append(f"resolution {fs/a.nfft:.1f} Hz, {a.nfft/fs*1000:.1f} ms segments ({per} per block), {dbs.shape[0]} segments")
    L.append("")
    L.append(f"strongest across 1-{fs/2000:.0f} kHz:")
    for i in allmarks:
        L.append(f"  {freqs[i]/1000:8.3f} kHz  {avg[i]:6.1f} dBFS")
    L.append("")
    L.append(f"{a.lo/1000:.1f}-{a.hi/1000:.1f} kHz, median floor {nf:.1f} dBFS:")
    seg_s = a.nfft / fs
    # 段與段的真實間隔：一塊只切得出 per 段，塊與塊之間還有掃描空隙，
    # 所以不是 nfft/fs。有 .TXT 就用實測的塊週期。
    period = np.diff(starts).mean() / 1e6 if len(starts) > 1 else (blocks.shape[1] + skip) / fs
    seg_dt = period / per
    for i in marks:
        tr = np.max(dbs[:, max(0, i - 1): i + 2], axis=1)
        # 強度怎麼變：標準差、在 1 Hz 附近有沒有週期（JJY/BPC 每秒一個符號）
        env = 10 ** (tr / 20)
        env = env - env.mean()
        spec = np.abs(np.fft.rfft(env))
        mf = np.fft.rfftfreq(len(env), seg_dt)
        j = np.argmax(spec[1:]) + 1 if len(spec) > 2 else 0
        swing = tr.max() - tr.min()
        mod = f"level swings {swing:.1f} dB, mostly at {mf[j]:.2f} Hz" if swing > 3 else "steady"
        L.append(f"  {freqs[i]/1000:8.3f} kHz  avg {avg[i]:6.1f}  min {tr.min():6.1f}  max {tr.max():6.1f} dBFS"
                 f"  | {mod}")
    L.append("")
    L.append("hints: JJY 40.000 / 60.000 kHz and BPC 68.500 kHz key the carrier once a second (level")
    L.append("modulation near 1 Hz). A steady line with no 1 Hz pattern is a local oscillator, a")
    L.append("switching supply, or an aliased MW station (see DEVLOG).")
    report = "\n".join(L)
    print(report)
    open(base + "_report.txt", "w", encoding="utf-8").write(report + "\n")

    plot_spectrum(base + "_spectrum.png", freqs, avg, a.lo, a.hi, marks)
    plot_wfall(base + "_wfall.png", freqs, dbs, a.lo, a.hi, seg_dt)
    print(f"\nwrote {base}_report.txt, _spectrum.png, _wfall.png")
    if a.iq:
        write_iq(base + "_iq.wav", blocks, fs, a.iq)
        print(f"wrote {base}_iq.wav (center {a.iq/1000:.3f} kHz, 50 ksps I/Q)")


if __name__ == "__main__":
    sys.exit(main())
