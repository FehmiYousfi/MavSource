include $(TOPDIR)/rules.mk
	PKG_NAME:=MavSource
	PKG_VERSION:=0
	PKG_RELEASE:=1
	PKG_BUILD_DIR:=$(PKG_NAME)-$(PKG_VERSION).$(PKG_RELEASE)

PKG_MAINTAINER:=Fehmi Yousfi <fehmi.yousfi@luceor.com>
PKG_LICENSE:=CC0-1.0

include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)/description
	Collector for PX4 Flight Controller multiple Serial Problem    
endef

define Package/$(PKG_NAME)
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:= MavlinkBridge plugin
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
	$(MAKE) -C $(PKG_BUILD_DIR)/src CC=$(TARGET_CC) CXX=$(TARGET_CXX) CONF_PATH=/etc/mavconfig/mavconf.conf
	cp $(PKG_BUILD_DIR)/src/MavSource  $(PKG_BUILD_DIR)/$(PKG_NAME)
	
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/bin/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/$(PKG_NAME) $(1)/bin/$(PKG_NAME)
endef

$(eval $(call BuildPackage,$(PKG_NAME),+libpthread,+libc,+libstdcpp))
