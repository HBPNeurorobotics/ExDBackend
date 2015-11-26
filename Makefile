#modules that have tests
TEST_MODULES=hbp_nrp_backend/hbp_nrp_backend/ hbp_nrp_cleserver/hbp_nrp_cleserver/ hbp_nrp_commons/hbp_nrp_commons/

#modules that are installable (ie: ones w/ setup.py)
INSTALL_MODULES=hbp-flask-restful-swagger-master hbp_nrp_backend hbp_nrp_commons hbp_nrp_cleserver

#packages to cover
COVER_PACKAGES=hbp_nrp_backend hbp_nrp_commons hbp_nrp_cleserver

#documentation to build
DOC_MODULES=hbp_nrp_backend/doc hbp_nrp_cleserver/doc hbp_nrp_commons/doc

##### DO NOT MODIFY BELOW #####################

CI_REPO?=ssh://bbpcode.epfl.ch/platform/ContinuousIntegration.git
CI_DIR?=ContinuousIntegration

FETCH_CI := $(shell \
		if [ ! -d $(CI_DIR) ]; then \
			git clone $(CI_REPO) $(CI_DIR) > /dev/null ;\
		fi;\
		echo $(CI_DIR) )
include $(FETCH_CI)/python/common_makefile
