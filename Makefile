build:
	docker build . -t capstone

run:
	docker run -p 4567:4567 -v $$PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it -v $$PWD:/capstone/ros capstone

