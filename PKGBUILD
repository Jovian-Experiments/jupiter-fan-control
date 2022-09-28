# Maintainer: Adam Weld <weld@valvesoftware.com>
# Maintainer: Christian Marcheselli <christianm@valvesoftware.com>

pkgname=jupiter-fan-control
_srcbranch=reduce_polling_frequency
pkgver=5.1
pkgrel=1
arch=('any')
license=('MIT')
pkgdesc="Jupiter fan controller"
url='https://gitlab.steamos.cloud/jupiter/jupiter-fan-control/-/tree/$_srcbranch'
depends=('python3' 'python-pyaml')
sha512sums=(SKIP)
makedepends=('git' 'rsync' 'openssh')

source=("git+ssh://git@gitlab.steamos.cloud/jupiter/jupiter-fan-control.git#branch=$_srcbranch")

package() {
   rsync -a --exclude 'README.md' "$srcdir"/jupiter-fan-control/* "$pkgdir"
}
