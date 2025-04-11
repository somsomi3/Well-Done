from flask import Flask, request, jsonify
import threading

app = Flask(__name__)

# Global reference to ROS node (set via function call)
node = None

def set_node_reference(node_instance):
    """Set reference to ROS node"""
    global node
    node = node_instance

@app.route('/command', methods=['POST'])
def receive_command():
    """Receive command from Spring server"""
    if node is None:
        return jsonify({'status': 'error', 'message': 'ROS node not initialized'}), 500
        
    command_data = request.json
    
    command_type = command_data.get('command')
    if not command_type:
        return jsonify({'status': 'error', 'message': 'Missing command type'}), 400
    
    # 명령 데이터를 명시적으로 딕셔너리로 변환
    command_dict = {
        'type': command_type
    }
    
    # 다른 필드 복사
    for key, value in command_data.items():
        if key != 'command':  # 'command'는 이미 'type'으로 처리했으므로 제외
            command_dict[key] = value
    
    # Add the command to the queue
    with node.queue_lock:
        node.command_queue.append(command_dict)
    
    return jsonify({'status': 'success', 'message': f'{command_type} command added to queue'})


@app.route('/auto-map', methods=['POST'])
def start_auto_map():
    """자동 매핑 시작 요청 처리"""
    if node is None:
        return jsonify({'status': 'error', 'message': 'ROS 노드가 초기화되지 않았습니다'}), 500
    
    # 요청 데이터 파싱
    request_data = request.json or {}
    data_value = request_data.get('data', True)  # 기본값은 True
    
    # 명령 큐에 추가
    with node.queue_lock:
        node.command_queue.append({
            'type': 'start_auto_map',
            'data': data_value
        })
    
    return jsonify({
        'status': 'success', 
        'message': f'자동 매핑 시작 명령이 큐에 추가되었습니다 (data={data_value})'
    })

# Add other routes here...

def run_flask_server():
    """Run Flask server"""
    app.run(host='0.0.0.0', port=5000)