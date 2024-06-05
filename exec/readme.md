# 빌드 및 배포 정리

## 사용 프로그램 및 프레임워크

### 배포 환경
- AWS EC2
    - UBUNTU 20.04.6(SSAFY 지급)
- MariaDB
    - Microsoft azure server(SSAFY 지급)
- Docker
    - Docker version 26.0.0
- Jenkins
    - Jenkins 2.440.2
    
### 프레임워크 및 모듈
- Server
    - Spring boot 3.2.4
    - JDK 17
- App
    - Android Studio Iguana 2023.2.1
- 자율주행
    - WSL Ubuntu 18.04
    - ROS melodic

## 포팅 메뉴얼

### 0. 배포에 앞서
- Gitlab에 프로젝트가 있는 상태에서, 프로젝트를 서버에 `git clone`으로 가져오거나 수동으로 배포하는 것이 아닌, Jenkins와 Docker를 이용하여 서비스를 배포하는 환경을 서술하였다.
- 다음 링크를 통해 개발 정보 등을 확인할 수 있으며, 특히 WSL 설치 및 세팅, ROS 환경 세팅은 해당하는 문서를 참고하여 설정해야 한다.
    - https://zenith-scapula-fc9.notion.site/2AM-e2add0786c5b41c89bd7c9356d109cce?pvs=4

### 1. 환경설정

- AWS EC2 ubuntu 20.04
    - 16GB memory
    - pem키와 도메인 주소를 지급받음
- project는 gitlab으로 관리중
- node.js를 배포할 목적(추후 웹 페이지 배포를 목적으로 수정할 예정)
- Docker를 설치하고, 컨테이너로 Jenkins를 설치하기
- Jenkins의 아이템은 Pipeline으로 구축

### 2. AWS 접속 환경 설정(with. Windows terminal)

(MobaXterm으로도 가능하므로, 각자 원하는 환경으로 접속  설정을 해주면 된다.)

- windows terminal 설치
- 터미널 탭에서 설정 → 새 프로필 추가
- 프로필 이름 설정
- `ssh -i "(pem키 경로)" (AWS 도메인orDNS 주소)`
    - ex) `ssh -i "C:\Users\SSAFY\Downloads\J10C110T.pem" [ubuntu@j10c110.p.ssafy.io](mailto:ubuntu@j10c110.p.ssafy.io)`
        - ubuntu 환경으로 서버를 생성했기 때문에 해당 도메인의 유저 기본 이름이 ubuntu이므로 `ubuntu@(IP or 도메인 주소)` 로 설정하면 된다

⇒ 정상적으로 설정되었다면 원격으로 서버에 접속한 터미널 창이 뜬다.

### 3. (선택사항) AWS EC2 메모리 용량 늘리기(swap 메모리 활용)

- 가상의 컨테이너를 실행하게 된다면 메모리를 많이 사용할 수 있음
- 프리 티어 환경에서는 Jenkins를 구동할 메모리가 부족할 수 있음
- 따라서 가상 메모리(swap 메모리)를 사용하여 임의로 메모리 볼륨을 늘림
- `free -h` : 메모리 확인
- `sudo fallocate -l 3G /swapfile` : 3G swap 메모리 생성 준비

```bash
sudo chmod 600 /swapfile # 권한 수정
sudo mkswap /swapfile    # 활성화 준비
sudo swapon /swapfile    # 활성화
```

+) 로컬에서 실행하는 우분투일 경우, 재부팅과 같은 작업이 있을 수 있다. 따라서 다음 작업도 수행한다(재부팅 시에도 swap 메모리를 사용하도록 설정)

```bash
sudo nano /etc/fstab # 파일 편집
```

```bash
## 내용 추가
/swapfile swap swap defaults 0 0
```

### 4. 서버에 Docker 설치

- [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/) 참고

```bash
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
```

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$UBUNTU_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### 5. Docker를 통해 Jenkins container 설치

- `sudo ufw` 로 포트 개방
    
    ※ 참고로 AWS에서 네트워크 단위로 포트가 열려있어야 ufw로 개방한 포트를 사용할 수 있음
    
    ```bash
    sudo ufw status #열려있는 포트 확인
    sudo ufw allow 22 #ssh 포트
    sudo ufw allow 8080 #Jenkins 포트
    sudo ufw show added #등록한 포트 조회
    sudo ufw enable #ufw 활성화
    ```
    
- Jenkins container 설치
    
    ※ 이 때, 생성한 Jenkins container에서 docker 환경을 사용하기 위해서는 로컬의 환경과 컨테이너의 환경을 매핑해줘야 한다.
    
    ```bash
    sudo docker run -itd \
    -p 8080:8080 \
    -p 50000:50000 \
    -v /home/ubuntu/jenkins-data:/var/jenkins_home \
    -v /$(which docker):/usr/bin/docker \
    -v /var/run/docker.sock:/var/run/docker.sock \
    --name jenkins jenkins/jenkins:lts
    ```
    
    `-i` : 컨테이너의 표준 입력 활성화
    
    `-t` : tty(가상 터미널) 할당
    
    위의 두 옵션은 주로 함께 사용(`-it`)
    
    `-d` : 컨테이너를 백그라운드로 실행
    
    `-p` : 호스트의 포트와 컨테이너의 포트를 연결(포트포워딩)
    
    `-v` : volume, <호스트경로:컨테이너 경로> 매핑으로 호스트 경로와 컨테이너 경로의 디렉토리를 연결
    
    `--name` : 컨테이너 이름 설정
    
    +) `--pivileged` : 컨테이너 안에서 호스트의 리눅스 커널 기능을 모두 사용
    
- 서버 아이피와 포트로 Jenkins 접속
    
    ex) j10c110.p.ssafy.io:8080
    
    - 이 때, Jenkins에서 패스워드를 요구한다. 패스워드는 서버 콘솔에서 `sudo docker logs jenkins` 를 입력해서 찾는다.
    - 이후 jenkins 설정을 마치고 접속하면 jenkins dashboard가 나온다.

### 6. Jenkins 설정하기

- Plugin 설치(Jenkins 관리)
    - docker 관련 플러그인 설치
        - Docker API Plugin
        - Docker Commons Plugin
        - Docker Pipeline
        - Docker plugin
    - NodeJS Plugin 설치
    - Gitlab 관련 플러그인 설치
        - Gitlab API Plugin
        - GitLab Authentication plugin
        - GitLab Branch Source Plugin
        - Gitlab Merge Request Builder
        - Generic Webhook Trigger Plugin
- Credentials 설정(Jenkins 관리)
    - Jenkins System의 Domains의 (global) 오른쪽 화살표나, global을 타고 들어가는 링크에 있는 곳에 Add Credentials 클릭
    - Kind : Username with password
    - Scope : Global
    - Username : Gitlab 계정 아이디
    - Password : Gitlab project token(모든 권한이 있는 maintenance 계정이어야 함)
    - ID, Description : 기호나 팀 컨벤션에 맞춰 작성
- Tools 설정(Jenkins 관리)
    - NodeJS를 쓸 것이므로, 설치한 NodeJS 플러그인을 바탕으로 NodeJS 설정을 진행한다
    - NodeJS 항목에서 설정
        - Name : 설정 이름, 필자는 nodejs-(version)으로 작성하였다.
            
            ex) nodejs-20.11
            
        - Install automatically 체크
        - 일반적으로 Install from nodejs.org로 설정되어 있을 것, 그렇지 않다면 하단의 Add Installer에서 설정해준다
        - Version : 사용할 nodejs 버전. 자신이 프로젝트를 구축한 환경의 nodejs 버전으로 설정해준다.
- 배포할 프로젝트(Gitlab)를 Pipeline으로 구축(새로운 Item)
    - item 이름 설정
    - Pipeline 타입으로 설정
    - Build Triggers 설정(이 때 Gitlab과 번갈아 작업해야함)
        - Jenkins : Build when a change is pushed to GitLab. 체크
        - Jenkins : 체크한 항목을 보면 GitLab webhook URL이 있는데 이 URL을 복사(만약 item 이름이 바뀌면 URL도 바뀐다)
        - GitLab : 진행중인 프로젝트의 Settings 항목의 Webhooks 선택(Maintainer 권한이 있는, 일반적으로 프로젝트를 생성한 팀장이 볼 수 있다. 다른 사람이 작업을 하기 위해서는 팀장이 Maintainer 권한을 부여해야 함)
        - GitLab : Add new webhook 클릭
        - Gitlab : URL에 복사한 URL을 붙여넣기
        - Jenkins : 고급 옵션을 펼쳐, 하단의 Secret token을 Generate한 후 복사
        - GitLab : Secret token에 생성한 토큰을 붙여넣기
        - GitLab : Trigger 항목에 Push event 체크, 여기서 All branches를 체크하여 모든 Push events에 대해 신호를 받거나, 또는 Wildcard pattern으로 특정 branch 신호만 받을 수 있다.
            
            wildcard pattern ex) feature/create_server
            
        - GitLab : 설정 저장
    - Pipeline 설정
        - Definition : Pipeline script from SCM 선택, 아래에 추가 항목 생성
        - SCM : Git 선택, 아래에 추가 항목 생성
        - Repositories에 배포할 Repository 생성하기
        - Repository URL : 프로젝트의 git URL 붙여넣기
        - Credentials : 앞에서 작성한 Credential을 설정(즉, 이 Credential은 해당 프로젝트를 관리하는 권한이 있는 계정임)
        - Branches to build : 신호를 받으면 실행할 브랜치를 작성(*/develop 으로 테스트해 볼 수도 있지 않을까?)
        - Repository browser는 자동에서 변경하지 않음
        - Script Path : Jenkinsfile
            
            ※ 프로젝트 최상위 디렉토리에 Jenkinsfile이라는 파일을 생성하면, 해당 파일에서 스크립트를 읽고 작업을 개시한다(Groovy 문법)
            
        - 설정을 저장

### 7. 배포 환경 설정

- 웹 서비스를 배포할 때는, 보안을 위해 서버에서 어떤 포트 번호를 쓰고 있는지 감출 필요가 있다.
- 필자는 reverse proxy라는 방법을 이용했는데, 아래는 reverse proxy를 적용한 과정을 서술한다.
- 아래의 과정은 백엔드 쪽에서 구동하는 서버로 연결했지만, 실제 서비스는 프론트에서 작업한 웹 페이지를 보여줘야 하므로, 프론트 웹 페이지가 보이도록 따로 작업할 필요가 있다.

```bash
sudo apt install nginx -y

sudo unlink /etc/nginx/sites-enabled/default
```

- `sudo vi /etc/nginx/sites-available/reverse-proxy.conf`를 입력하고 아래의 내용을 작성한다.

```bash
server {
        listen 80;
        listen [::]:80;
        server_name url;
				#url은 자신의 aws 주소를 입력한다

        access_log /var/log/nginx/reverse-access.log;
        error_log /var/log/nginx/reverse-error.log;

        location / {
                    proxy_pass http://127.0.0.1:{포트번호};
										#포트번호는 서버를 개방하기 위해 설정한 포트번호를 입력한다
  }
}
```

- `sudo vi /etc/nginx/sites-enabled/reverse-proxy.conf` 에서도 마찬가지로 작성한다
- nginx를 사용하기 위해 포트를 개방한다

```bash
sudo ufw allow 80
#http://
sudo ufw allow 443
#https://
sudo ufw enable
```

- 작성한 nginx 설정이 적용되었는지 확인한다

```bash
sudo nginx -t
```

- nginx를 재시작한다

```bash
sudo service nginx restart
```

- 위 설정을 완료하면, AWS의 주소로 바로 접속했을 때(실제로는 80포트나 443 포트로 접속하는 것) 해당 포트가 가리키는 서버 포트로 접속한다.

- 위의 reverse proxy는 개인적으로 더 공부한 다음 다른 방법으로 어떻게 적용하는지 파악할 필요가 있을 듯함
- 프로젝트를 진행하는 과정에서, 생각만 하고 미처 적용하지 못한 HTTPS 보안 프로토콜을 적용해 보았다.
- 아래는 SSL 인증서를 발급받고 HTTPS를 사용하는 방법이다. 먼저 certbot을 설치한다.

```bash
sudo apt-get update
sudo apt-get install certbot python3-certbot-nginx
```

- 그 다음 SSL 인증서를 발급받는다.

```bash
sudo certbot --nginx -d (domain 주소)
```

- SSAFY 특화 프로젝트를 진행할 때는 다음과 같이 진행했다.

```bash
sudo certbot --nginx -d j10c110.p.ssafy.io
```

- 위 명령어를 입력하면 절차를 거쳐 보안 프로토콜을 입력하게 되는데, 중간에 HTTP를 HTTPS로 리다이렉팅 할 것인지 물어본다. 자신이 배포할 환경에 맞춰서 리다이렉팅을 할 것인지 결정하면 된다.
- 이번 배포에서는 같은 443 포트에서, /api가 붙는 주소는 server, 그렇지 않으면 client로 연결되도록 reverse-proxy 설정을 진행했다.
- 먼저 /etc/nginx/nginx.conf 파일을 다음과 같이 작성했다. 중간에 보이는 볼드체가 기존 설정에서 추가한 부분이다.
```bash
user www-data;
worker_processes auto;
pid /run/nginx.pid;
include /etc/nginx/modules-enabled/*.conf;

events {
        worker_connections 768;
        # multi_accept on;
}

http {

        ##
        # Basic Settings
        ##

        sendfile on;
        tcp_nopush on;
        tcp_nodelay on;
        keepalive_timeout 65;
        types_hash_max_size 2048;
        # server_tokens off;

        # server_names_hash_bucket_size 64;
        # server_name_in_redirect off;

        include /etc/nginx/mime.types;
        default_type application/octet-stream;

        ##
        # SSL Settings
        ##

        ssl_protocols TLSv1 TLSv1.1 TLSv1.2 TLSv1.3; # Dropping SSLv3, ref: POODLE
        ssl_prefer_server_ciphers on;

        ##
        # Logging Settings
        ##

        access_log /var/log/nginx/access.log;
        error_log /var/log/nginx/error.log;

        ##
        # Gzip Settings
        ##

        gzip on;

        # gzip_vary on;
        # gzip_proxied any;
        # gzip_comp_level 6;
        # gzip_buffers 16 8k;
        # gzip_http_version 1.1;
        # gzip_types text/plain text/css application/json application/javascript text/xml application/xml application/xml+rss text/javascript;

        ##
        # Virtual Host Configs
        ##

        include /etc/nginx/conf.d/*.conf;
        include /etc/nginx/sites-enabled/*;

        # this is new configuration code from 2024-03-27

        **# setting BACKEND
        upstream backend {
                server 127.0.0.1:3000;
        }

        # setting FRONTEND
        upstream frontend {
                server 127.0.0.1:5173;**
        }
}

#mail {
#       # See sample authentication script at:
#       # http://wiki.nginx.org/ImapAuthenticateWithApachePhpScript
#
#       # auth_http localhost/auth.php;
#       # pop3_capabilities "TOP" "USER";
#       # imap_capabilities "IMAP4rev1" "UIDPLUS";
#
#       server {
#               listen     localhost:110;
#               protocol   pop3;
#               proxy      on;
#       }
#
#       server {
#               listen     localhost:143;
#               protocol   imap;
#               proxy      on;
#       }
#}
```
    
- 다음으로 reverse-proxy.conf 파일을 다음과 같이 작성했다.
    
    ```bash
    server {
            server_name j10c110.p.ssafy.io;
    
            access_log /var/log/nginx/reverse-access.log;
            error_log /var/log/nginx/reverse-error.log;
    
            location / {
                    proxy_pass http://frontend/;
            }
    
            location /api {
                    proxy_pass http://backend/api;
            }
    
        listen [::]:443 ssl ipv6only=on; # managed by Certbot
        listen 443 ssl; # managed by Certbot
        ssl_certificate /etc/letsencrypt/live/j10c110.p.ssafy.io/fullchain.pem; # managed by Certbot
        ssl_certificate_key /etc/letsencrypt/live/j10c110.p.ssafy.io/privkey.pem; # managed by Certbot
        include /etc/letsencrypt/options-ssl-nginx.conf; # managed by Certbot
        ssl_dhparam /etc/letsencrypt/ssl-dhparams.pem; # managed by Certbot
    
    }
    server {
        if ($host = j10c110.p.ssafy.io) {
            return 301 https://$host$request_uri;
        } # managed by Certbot
    
            listen 80;
            listen [::]:80;
            server_name j10c110.p.ssafy.io;
        return 404; # managed by Certbot
    }
    ```
    
- 위처럼 작성한 경우, 일반적인 도메인 접근은 client port로, /api로의 접근은 server port로 연결된다.
- 이후 `sudo nginx -t` 로 설정 파일을 검사한 다음, 문제가 없다면 `sudo service nginx restart` 명령으로 nginx를 재시작한다.
