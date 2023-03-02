scp -r admin@10.29.30.2:/var/log/game_logs $HOME\game_logs
ssh admin@10.29.30.2 rm /var/log/game_logs/*
sleep 2