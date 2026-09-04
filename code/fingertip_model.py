import cv2
import numpy as np
import torch
import torch.nn as nn

INPUT_W = 256
INPUT_H = 192
STRIDE = 8
HEATMAP_W = INPUT_W // STRIDE
HEATMAP_H = INPUT_H // STRIDE

CHANNELS = (24, 48, 96, 64)
DILATIONS = (1, 2, 4, 8)

DEFAULT_CONF_THRESHOLD = 0.35


def _conv_bn(cin, cout, pool=False, dilation=1):
    layers = [
        nn.Conv2d(cin, cout, kernel_size=3, padding=dilation, dilation=dilation),
        nn.BatchNorm2d(cout),
        nn.ReLU(inplace=True),
    ]
    if pool:
        layers.append(nn.MaxPool2d(2))
    return nn.Sequential(*layers)


class TinyFCN(nn.Module):

    def __init__(self):
        super().__init__()
        c1, c2, c3, c4 = CHANNELS
        blocks = [
            _conv_bn(1, c1, pool=True),
            _conv_bn(c1, c2, pool=True),
            _conv_bn(c2, c3, pool=True),
        ]
        cin = c3
        for d in DILATIONS:
            blocks.append(_conv_bn(cin, c4, dilation=d))
            cin = c4
        blocks.append(nn.Conv2d(c4, 1, kernel_size=1))
        self.net = nn.Sequential(*blocks)

    def forward(self, x):
        return self.net(x)    


def preprocess(img_bgr: np.ndarray) -> torch.Tensor:
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (INPUT_W, INPUT_H), interpolation=cv2.INTER_AREA)
    t = torch.from_numpy(resized).float().div_(255.0)
    return t.unsqueeze(0).unsqueeze(0)


def soft_argmax_2d(heatmap: np.ndarray, radius: int = 2):
    h, w = heatmap.shape
    flat_idx = int(np.argmax(heatmap))
    py, px = divmod(flat_idx, w)

    y0, y1 = max(0, py - radius), min(h, py + radius + 1)
    x0, x1 = max(0, px - radius), min(w, px + radius + 1)
    patch = heatmap[y0:y1, x0:x1]

    ys, xs = np.mgrid[y0:y1, x0:x1]
    weight_sum = patch.sum()
    if weight_sum <= 1e-6:
        return float(px), float(py), float(heatmap[py, px])

    x_refined = float((xs * patch).sum() / weight_sum)
    y_refined = float((ys * patch).sum() / weight_sum)
    return x_refined, y_refined, float(heatmap[py, px])


class FingertipDetector:

    def __init__(self, weights_path: str, conf_threshold: float = DEFAULT_CONF_THRESHOLD,
                 device: str = "cpu"):
        self.device = torch.device(device)
        self.model = TinyFCN().to(self.device)
        state = torch.load(weights_path, map_location=self.device)
        try:
            self.model.load_state_dict(state)
        except RuntimeError as e:
            raise SystemExit(
                f"Wagi w {weights_path} nie pasuja do biezacej architektury sieci.\n"
                f"Jesli trenowales model na starszej wersji (wejscie 128x128), "
                f"przetrenuj go ponownie:\n"
                f"    python train_heatmap_model.py --dataset dataset\n"
                f"Zebrany dataset (images/ + labels.json) pozostaje wazny - "
                f"etykiety sa skalowane automatycznie.\n\nSzczegoly: {e}")
        self.model.eval()
        self.conf_threshold = conf_threshold

    @torch.no_grad()
    def predict(self, img_bgr: np.ndarray):
        h, w = img_bgr.shape[:2]
        x = preprocess(img_bgr).to(self.device)
        logits = self.model(x)
        heatmap = torch.sigmoid(logits)[0, 0].cpu().numpy()

        hx, hy, confidence = soft_argmax_2d(heatmap)

        x_in = (hx + 0.5) * STRIDE
        y_in = (hy + 0.5) * STRIDE
        px = int(x_in * (w / INPUT_W))
        py = int(y_in * (h / INPUT_H))

        found = confidence >= self.conf_threshold and 0 <= px < w and 0 <= py < h
        return found, px, py, confidence
