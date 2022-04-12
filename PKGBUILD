# Maintainer: Adam Weld <weld@valvesoftware.com>

pkgname=jupiter-fan-controller
pkgver=0.0.2
pkgrel=1
arch=('any')
license=('GPLv3')
depends=('python3' 'python-pyaml')
source=("fancontrol.py" "PID.py" "jupiter-fan-control.service" "jupiter-fan-control-config.yaml")
# skipping sha256 sums for dev iteration - no 'makepkg -g' needed
package() {
  pwd
  install -m 644 -Dt "$pkgdir/usr/lib/systemd/system/" \
    "${srcdir}"/jupiter-fan-control.service 
  install -m 755 -Dt "$pkgdir/usr/share/$pkgname/" \
    "${srcdir}"/fancontrol.py \
    "${srcdir}"/PID.py
  install -m 755 -Dt "$pkgdir/etc/" \
    "${srcdir}"/jupiter-fan-control-config.yaml
}
sha256sums=('ddebcfe9395a132c60516ed05d272fd628e2e87d6b0ec09e128a2a9eea5fa643'
            '0aa8264f397d850814919d1d34bb604d6c422d44ede43c3022d4a361616b3adb'
            'c7ddee015b4d3664d9b6be7af70929f7084f6b1143bdcf97a6275c9c5ac90c3d'
            '59374bd6904d86bbc10d5bc6531d7ca462a9502a2bd6f19fe0d37a7989ac27da')
