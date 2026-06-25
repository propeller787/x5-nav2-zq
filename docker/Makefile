# =============================================================================
# Nav2 MPPI Docker 镜像管理 Makefile
# 用法:
#   make build        构建镜像
#   make run          启动开发容器 (挂载代码)
#   make exec         进入运行中的容器
#   make save         导出为 tar.gz (给同事离线分发)
#   make load         从 tar.gz 加载镜像
#   make push         推送到阿里云 registry
#   make pull         从阿里云 pull
#   make clean        清理
# =============================================================================

IMAGE    := nav2_mppi:dev
TAR      := nav2_mppi_dev.tar.gz
REGISTRY := registry.cn-shanghai.aliyuncs.com/howard/nav2_mppi
REMOTE   := $(REGISTRY):dev

.PHONY: build run exec save load push pull clean logs stop

build:
	docker build --no-cache -t $(IMAGE) -f docker/Dockerfile .

build-cache:
	docker build -t $(IMAGE) -f docker/Dockerfile .

run:
	docker-compose -f docker/docker-compose.yml run --rm nav2

# 在跑容器时用
exec:
	docker exec -it nav2_mppi_dev bash

# 查日志
logs:
	docker logs -f nav2_mppi_dev

# 停掉运行中的容器
stop:
	docker stop nav2_mppi_dev 2>/dev/null || true

# ------------------------------------------------------------------
# 离线分发 (U盘/网盘/飞书, 适合一次性给同事)
# ------------------------------------------------------------------
save:
	@echo "[1/2] 导出镜像..."
	docker save $(IMAGE) | gzip > $(TAR)
	@echo "[2/2] 完成"
	@ls -lh $(TAR)

load:
	@echo "[1/2] 加载镜像..."
	@if [ -f $(TAR) ]; then \
		docker load -i $(TAR); \
	else \
		echo "错误: $(TAR) 不存在"; \
		exit 1; \
	fi
	@echo "[2/2] 完成"
	@docker images $(IMAGE)

# ------------------------------------------------------------------
# 推送到阿里云 (团队协作推荐, 改了代码再 build+push 即可)
# 用法: make push REGISTRY=registry.cn-shanghai.aliyuncs.com/your_ns/nav2_mppi
# ------------------------------------------------------------------
push:
	@echo "[1/3] tag..."
	docker tag $(IMAGE) $(REMOTE)
	@echo "[2/3] login (输入阿里云密码)..."
	@docker login registry.cn-shanghai.aliyuncs.com -u howard
	@echo "[3/3] push..."
	docker push $(REMOTE)

pull:
	docker pull $(REMOTE)
	docker tag $(REMOTE) $(IMAGE)

# ------------------------------------------------------------------
# 清理
# ------------------------------------------------------------------
clean:
	rm -f $(TAR)
	docker rm -f nav2_mppi_dev 2>/dev/null || true
	docker rmi $(IMAGE) 2>/dev/null || true
