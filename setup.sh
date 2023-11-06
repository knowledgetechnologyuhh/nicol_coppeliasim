#!/bin/bash
#git checkout hri_demo_lab
python3 -m venv NICOL-env
source NICOL-env/bin/activate

pip install wheel setuptools pip --upgrade

#torch==1.12.1
#torchaudio==0.12.1

#pip install torch==1.12.1+cu116 torchvision==0.13.1+cu116 torchaudio==0.12.1 numpy==1.21.6 --extra-index-url https://download.pytorch.org/whl/cu116
#pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

#mkdir dependencies
#cd dependencies
#git clone https://github.com/UM-ARM-Lab/pytorch_kinematics.git
#cd pytorch_kinematics
#git checkout bee97ab0d20d21145df2519b1a6db346f20f78d4
#pip install -e .
#cd ../..

pip install -r requirements.txt

#echo "instaling cykleik"
#git clone https://git.informatik.uni-hamburg.de/jangerritha/cycleik
#cd cycleik
#git checkout dev
#pip install -e .
#cd ..

echo -n "Install nicol_talker for Text to speach support? [y/N] "
read VAR
if [[ $VAR = "y" ]]
then
    mkdir dependencies
    cd dependencies
    echo "Installing nicol_tts..."
    git clone https://github.com/knowledgetechnologyuhh/nicol_tts.git
    cd nicol_tts
    pip install -r requirements.txt
    cp -r tts_models--en--vctk--vits ../..
    cd ../..
fi

sudo apt-get install espeak

#cp -r resources/cycleik_weights/* dependencies/cycleik
#ln ~/NICOL-env/bin/activate .
#source activate
