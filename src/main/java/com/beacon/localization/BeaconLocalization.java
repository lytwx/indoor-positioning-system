package com.beacon.localization;

class Beacon {
    double x, y;     // 坐标
    double rssi;     // 接收的信号强度

    public Beacon(double x, double y, double rssi) {
        this.x = x;
        this.y = y;
        this.rssi = rssi;
    }

    // RSSI → 距离估算，使用 log-distance 路径损耗模型
    public double estimateDistance(double txPower, double pathLossExp) {
        return Math.pow(10.0, (txPower - rssi) / (10.0 * pathLossExp));
    }
}

public class BeaconLocalization {

    // 误差函数 E(x, y)：点 (x, y) 到每个圆周距离与估算半径的误差平方和
    static double computeError(double x, double y, Beacon[] beacons, double txPower, double pathLossExp) {
        double error = 0.0;
        for (Beacon b : beacons) {
            double estDist = b.estimateDistance(txPower, pathLossExp);
            double dx = x - b.x;
            double dy = y - b.y;
            double dist = Math.sqrt(dx * dx + dy * dy);
            error += Math.pow(dist - estDist, 2);
        }
        return error;
    }

    // 梯度下降法求近似交点
    static double[] gradientDescent(Beacon[] beacons, double txPower, double pathLossExp) {
        double x = 0, y = 0;
        for (Beacon b : beacons) {
            x += b.x;
            y += b.y;
        }
        x /= beacons.length;
        y /= beacons.length;

        double learningRate = 0.01;
        int maxIter = 10000;

        for (int iter = 0; iter < maxIter; iter++) {
            double gradX = 0, gradY = 0;
            for (Beacon b : beacons) {
                double estDist = b.estimateDistance(txPower, pathLossExp);
                double dx = x - b.x;
                double dy = y - b.y;
                double dist = Math.sqrt(dx * dx + dy * dy);
                if (dist == 0) continue;
                double error = dist - estDist;
                gradX += 2 * error * dx / dist;
                gradY += 2 * error * dy / dist;
            }
            x -= learningRate * gradX;
            y -= learningRate * gradY;
        }

        return new double[]{x, y};
    }

    public static void main(String[] args) {
        // 假设 txPower = -59dBm（1米距离），pathLossExp = 2.0（自由空间）
        double txPower = -59;
        double pathLossExp = 2.0;

        // 三个 iBeacon 的坐标和 RSSI 值（可以从实际设备获得）
        Beacon[] beacons = new Beacon[]{
            new Beacon(200, 100, -65),
            new Beacon(200, 250, -70),
            new Beacon(300, 150, -60)
        };

        double[] position = gradientDescent(beacons, txPower, pathLossExp);
        System.out.printf("Estimated position: (%.4f, %.4f)\n", position[0], position[1]);
    }
}
