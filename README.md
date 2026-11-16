git config --global --unset http.proxy
git config --global --unset https.proxy
grep -rn "2080" ~/.bashrc ~/.profile ~/.zshrc /etc/environment 2>/dev/null

git config --list --show-origin | grep -i proxy
git config --local --unset http.proxy
git config --local --unset https.proxy
sudo git config --system --unset http.proxy
sudo git config --system --unset https.proxy
env | grep -i proxy
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY