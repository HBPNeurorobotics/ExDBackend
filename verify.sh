#!/bin/bash

# ------------------------------------------------------------------
# Simple wrapper around the "make verify" target of the common hbp 
# Makefile for python. This is mainly use for local development.
# ------------------------------------------------------------------

read -d '' USAGE << EOF
Usage: ./verify.sh -hr\n
r     Try to release the code to HBP pypy server\n
h     Print usage

EOF

while getopts ":rh" optname
  do
    case "$optname" in
      "r")
        RELEASE=true
        ;;
      "h")
        echo "$USAGE"
        exit 0;
        ;;
      "?")
        echo "Unknown option $OPTARG"
        echo "$USAGE"
        exit 0;
        ;;
      *)
        echo "Unknown error while processing options"
        exit 0;
        ;;
    esac
  done

# Note: Dont put CLE into this list. otherwiese the files CLELauncher, ROSCLEServer and others will not be pep8 and pylint validated! 
export IGNORE_LINT="platform_venv|hbp_nrp_backend/hbp_nrp_backend/exd_config/generated|hbp_nrp_commons/hbp_nrp_commons/generated|hbp-flask-restful-swagger-master|GazeboRosPackages|migrations|build"

rm hbp_nrp_backend/hbp_nrp_backend/bibi_config/tests/generated_cle_script.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment_bibi.py

sed -ibak -f version.txt hbp_nrp_backend/hbp_nrp_backend/version.py hbp_nrp_backend/requirements.txt
sed -ibak -f version.txt hbp_nrp_commons/hbp_nrp_commons/version.py hbp_nrp_commons/requirements.txt
sed -ibak -f version.txt hbp_nrp_cleserver/hbp_nrp_cleserver/version.py hbp_nrp_cleserver/requirements.txt

if [ "$RELEASE" = true ] ; then
    make verify
else
	make verify_base
fi
VERIFY_RET=$?

mv hbp_nrp_backend/hbp_nrp_backend/version.pybak hbp_nrp_backend/hbp_nrp_backend/version.py -f
mv hbp_nrp_commons/hbp_nrp_commons/version.pybak hbp_nrp_commons/hbp_nrp_commons/version.py -f
mv hbp_nrp_cleserver/hbp_nrp_cleserver/version.pybak hbp_nrp_cleserver/hbp_nrp_cleserver/version.py -f
mv hbp_nrp_backend/requirements.txtbak hbp_nrp_backend/requirements.txt -f
mv hbp_nrp_commons/requirements.txtbak hbp_nrp_commons/requirements.txt -f
mv hbp_nrp_cleserver/requirements.txtbak hbp_nrp_cleserver/requirements.txt -f

exit $VERIFY_RET
