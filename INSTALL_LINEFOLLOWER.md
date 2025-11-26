# ライントレーサー デーモン インストール手順

## 機能

| 操作 | 動作 |
|------|------|
| **電源ON** | 自動で待機状態 |
| **ボタン短押し** | スタート / ストップ 切り替え |
| **ボタン長押し（2秒）** | シャットダウン |

## Raspberry Pi へのインストール

### 1. ファイルをRaspberry Piにコピー

```bash
# GitHubからクローン（または手動でコピー）
cd ~
git clone https://github.com/takenoko888/EicDesignLab.git
```

### 2. プログラムをインストール

```bash
# プログラムをコピー
sudo cp ~/EicDesignLab/eiclab_linefollower_button.py /usr/local/sbin/
sudo chmod +x /usr/local/sbin/eiclab_linefollower_button.py
```

### 3. サービスをインストール

```bash
# サービスファイルをコピー
sudo cp ~/EicDesignLab/services/linefollower.service /etc/systemd/system/

# サービスを有効化（起動時に自動実行）
sudo systemctl daemon-reload
sudo systemctl enable linefollower.service

# サービスを開始
sudo systemctl start linefollower.service
```

### 4. 動作確認

```bash
# サービスの状態確認
sudo systemctl status linefollower.service

# ログ確認
journalctl -u linefollower.service -f
```

## アンインストール

```bash
# サービス停止・無効化
sudo systemctl stop linefollower.service
sudo systemctl disable linefollower.service

# ファイル削除
sudo rm /etc/systemd/system/linefollower.service
sudo rm /usr/local/sbin/eiclab_linefollower_button.py
sudo systemctl daemon-reload
```

## トラブルシューティング

### サービスが起動しない場合

```bash
# エラーログを確認
journalctl -u linefollower.service -n 50

# 手動で実行してエラーを確認
sudo python3 /usr/local/sbin/eiclab_linefollower_button.py
```

### ピン設定の確認

```
PIN_BUTTON = 3   (GPIO3)
PIN_LED = 23     (GPIO23) ※オプション
PIN_AIN1 = 6     (GPIO6)  モーター左 前進
PIN_AIN2 = 5     (GPIO5)  モーター左 後退
PIN_BIN1 = 26    (GPIO26) モーター右 前進
PIN_BIN2 = 27    (GPIO27) モーター右 後退
MCP3004: SPI (ch0-3)
```
