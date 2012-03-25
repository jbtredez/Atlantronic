# Copyright 1999-2007 Gentoo Foundation
# Distributed under the terms of the GNU General Public License v2

inherit autotools eutils git-2

DESCRIPTION="Atlantronic usb module"
HOMEPAGE="http://www.atlantronic.fr"
SRC_URI=""
EGIT_REPO_URI="git://github.com/jbtredez/Atlantronic.git"
EGIT_MASTER="jb"

LICENSE="GPL-2"
KEYWORDS="amd64 x86"
SLOT="0"
IUSE=""

DEPEND=""

src_compile()
{
	emake ARCH=`uname -m` modules || die "make failed"
}


src_install()
{
	make DESTDIR="${D}" ARCH=`uname -m` install || die "make install failed"
}
