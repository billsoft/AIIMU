import asyncio
import serial_asyncio


async def read_serial(reader, buffer_queue):
    """读取串口数据并将其添加到缓冲队列中"""
    while True:
        try:
            data = await reader.read(100)
            if data:
                buffer_queue.put_nowait(data)
                print(f'Received: {data}')
                await asyncio.sleep(0.01)  # 添加10毫秒的休眠
            else:
                print("No data received. Closing read_serial.")
                break
        except asyncio.CancelledError:
            print("read_serial task cancelled.")
            break
        except Exception as e:
            print(f"Error in read_serial: {e}")
            break


async def write_serial_user_input(writer):
    """异步读取用户输入并发送到串口"""
    loop = asyncio.get_running_loop()
    while True:
        try:
            # 使用run_in_executor避免阻塞事件循环
            message = await loop.run_in_executor(None, input, "Enter message: ")
            writer.write(message.encode('utf-8'))
            await writer.drain()
            print(f'Sent (user input): {message}')
        except asyncio.CancelledError:
            print("write_serial_user_input task cancelled.")
            break
        except Exception as e:
            print(f"Error in write_serial_user_input: {e}")
            break


async def write_serial_loop(writer):
    """循环发送预定义的报文序列"""
    # 定义报文序列和对应的等待时间（秒）
    sequences = [
        # 第一组报文
        [
            bytes.fromhex('41 54 a0 07 eb f4 08 00 00 00 00 00 00 00 00 0d 0a'),
        ],
        0.5,  # 等待0.5秒

        [
            bytes.fromhex('41 54 90 07 eb f4 08 05 70 00 00 04 00 00 00 0d 0a'),
        ],
        0.5,  # 等待0.5秒
        [
            bytes.fromhex('41 54 18 07 eb f4 08 00 00 00 00 00 00 00 00 0d 0a'),
        ],
        2,  # 等待0.5秒

        [
            bytes.fromhex('41 54 20 07 eb f4 08 00 00 00 00 00 00 00 00 0d 0a'),
        ],
        0.5,  # 等待0.5秒

        [
            bytes.fromhex('41 54 90 07 eb f4 08 05 70 00 00 01 00 00 00 0d 0a'),
        ],
        0.5,
        [
            bytes.fromhex('41 54 18 07 eb f4 08 00 00 00 00 00 00 00 00 0d 0a'),
        ],
        0.5,

        [
            bytes.fromhex('41 54 90 07 eb f4 08 17 70 00 00 00 00 a0 40 0d 0a'),
        ],
        0.5,
        [
            bytes.fromhex('41 54 90 07 eb f4 08 16 70 00 00 9a 99 99 be 0d 0a'),
        ],
        1,

        [
            bytes.fromhex('41 54 90 07 eb f4 08 17 70 00 00 00 00 a0 40 0d 0a'),
        ],
        0.5,
        [
            bytes.fromhex('41 54 90 07 eb f4 08 16 70 00 00 9a 99 99 be 0d 0a'),
        ],
        1,
    ]

    while True:
        for item in sequences:
            if isinstance(item, list):
                for msg in item:
                    try:
                        writer.write(msg)
                        await writer.drain()
                        print(f'Sent (loop): {msg.hex()}')
                        await asyncio.sleep(0.01)  # 可选的短暂延迟
                    except asyncio.CancelledError:
                        print("write_serial_loop task cancelled during sending messages.")
                        return
                    except Exception as e:
                        print(f"Error sending message in write_serial_loop: {e}")
                        return
            elif isinstance(item, (int, float)):
                try:
                    await asyncio.sleep(item)
                except asyncio.CancelledError:
                    print("write_serial_loop task cancelled during sleep.")
                    return
                except Exception as e:
                    print(f"Error during sleep in write_serial_loop: {e}")
                    return
            else:
                print(f"Unknown item in sequences: {item}")


async def process_queue(buffer_queue):
    """处理从缓冲队列中取出的数据"""
    while True:
        try:
            data = await buffer_queue.get()
            print(f'Processing: {data}')
            # 在这里添加你对数据的具体处理逻辑
            buffer_queue.task_done()
        except asyncio.CancelledError:
            print("process_queue task cancelled.")
            break
        except Exception as e:
            print(f"Error in process_queue: {e}")
            break


async def main(buffer_queue):
    """主协程，初始化串口连接并启动各个任务"""
    try:
        reader, writer = await serial_asyncio.open_serial_connection(
            url='COM5', baudrate=921600)
        print("Serial port COM5 opened successfully.")
    except Exception as e:
        print(f'Failed to open serial port: {e}')
        return

    # 创建各个任务
    read_task = asyncio.create_task(read_serial(reader, buffer_queue))
    write_task_input = asyncio.create_task(write_serial_user_input(writer))
    write_task_loop = asyncio.create_task(write_serial_loop(writer))
    process_task = asyncio.create_task(process_queue(buffer_queue))

    # 等待所有任务完成（通常不会发生，除非有任务异常退出）
    done, pending = await asyncio.wait(
        [read_task, write_task_input, write_task_loop, process_task],
        return_when=asyncio.FIRST_EXCEPTION
    )

    for task in done:
        if task.exception():
            print(f"Task {task} raised an exception: {task.exception()}")

    for task in pending:
        task.cancel()


if __name__ == '__main__':
    # 创建一个共享的缓冲队列
    buffer_queue = asyncio.Queue()

    try:
        asyncio.run(main(buffer_queue))
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
