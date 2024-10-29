include $(TOPDIR)/rules.mk
	PKG_NAME:=MAVRouterPorted
	PKG_VERSION:=0
	PKG_RELEASE:=1
	PKG_BUILD_DIR:=$(PKG_NAME)-$(PKG_VERSION).$(PKG_RELEASE)

PKG_MAINTAINER:=Fehmi Yousfi <fehmi.yousfi@luceor.com>
PKG_LICENSE:=CC0-1.0


include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)/description
	a bridge for mavlink data from UART to UDP link on openwrt : test target BCM2711 
endef

define Package/$(PKG_NAME)
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:= MavlinkBridge for advanced teleemtry purposes
	DEPENDS:=+libc +libpthread +libstdcpp


endef

define Build/Clean
	$(MAKE) clean  
endef

define Build/Configure
    # Empty body to skip the configure step
endef


define Build/Prepare
	if [ ! -d $(PKG_BUILD_DIR) ]; then \
		mkdir -p $(PKG_BUILD_DIR); \
	elif [ -n "$(ls -A $(PKG_BUILD_DIR))" ]; then \
		rm -r $(PKG_BUILD_DIR); \
		mkdir -p $(PKG_BUILD_DIR); \
	else \
		rm -r $(PKG_BUILD_DIR); \
		mkdir -p $(PKG_BUILD_DIR); \
	fi
	$(CP) ./src* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR)/src CC=$(TARGET_CC) CXX=$(TARGET_CXX)
	cp $(PKG_BUILD_DIR)/src/mavlink-routerd  $(PKG_BUILD_DIR)/$(PKG_NAME)
	
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/bin/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/$(PKG_NAME) $(1)/bin/$(PKG_NAME)

	$(INSTALL_DIR) $(1)/etc/mavconfig	
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_DIR) $(1)/etc/config

	$(INSTALL_BIN) files/etc/init.d/mavrouter $(1)/etc/init.d/mavrouter
	$(INSTALL_DATA) files/etc/config/mavconf.conf $(1)/etc/mavconfig/mavconf.conf
	$(INSTALL_DATA) files/etc/config/mavhost $(1)/etc/config/mavhost
endef

$(eval $(call BuildPackage,$(PKG_NAME),+libpthread,+libc,+libstdcpp))



