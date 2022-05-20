# Maintainer: Adam Weld <weld@valvesoftware.com>

pkgname=jupiter-fan-control
pkgver=0.0.3
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
    "${srcdir}"/PID.py \
    "${srcdir}"/jupiter-fan-control-config.yaml
}
sha256sums=('SKIP'
            'SKIP'
            'SKIP'
            'SKIP')
