## BQ34Z100-G1 Fuel Gauge
BQ34Z100-G1 Wide Range Fuel Gauge with Impedance Track™ Technology

Datasheet : https://www.ti.com/lit/ds/symlink/bq34z100-g1.pdf?ts=1646040278028&ref_url=https%253A%252F%252Fwww.google.com%252F

Reference Manual : https://www.ti.com/lit/ug/sluubw5a/sluubw5a.pdf?ts=1652085611421&ref_url=https%253A%252F%252Fwww.google.com%252F

## HOW TO COMPILE
```
make
```
## EXECUTE THE BINARY
```
./battery_parameters
```
## ADD SYSTEMD SERVICE AND ENABLE IT
```
systemctl enable battery_parameters
systemctl start battery_parameters
```

## HOW TO READ BATTERY PARAMETERS VALUES
```
cat /tmp/battery_parameters
```
