// Cargo.toml dependencies:
// [dependencies]
// eframe = { version = "0.24", features = ["default"] }
// egui = "0.24"
// serialport = "4.2"
// egui_plot = "0.24"
// chrono = "0.4"
//
// [target.'cfg(windows)'.dependencies]
// winapi = { version = "0.3", features = ["winuser", "windef"] }

use eframe::egui;
use egui::{Color32, RichText, Stroke};
use egui_plot::{Line, Plot, PlotPoints, Legend, Corner};
use std::collections::HashMap;
use std::fs::File;
use std::io::Write;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use chrono::Local;

#[derive(Clone)]
struct DataPoint {
    timestamp: f64,
    values: HashMap<String, f64>,
    raw_text: String,
}

#[derive(Clone)]
struct EnvironmentData {
    timestamp: f64,
    // BME688 - Gas Sampling Chamber
    sample_temp: f64,
    sample_humid: f64,
    // AHT25 - Object Chamber
    chamber_temp: f64,
    chamber_humid: f64,
}

struct ChannelData {
    points: Vec<(f64, f64)>,
    color: Color32,
    visible: bool,
    min_value: f64,
    max_value: f64,
    avg_value: f64,
}

struct SerialMonitorApp {
    available_ports: Vec<String>,
    selected_port: String,
    baud_rate: String,
    is_connected: bool,
    received_data: Arc<Mutex<Vec<String>>>,
    plot_data: Arc<Mutex<Vec<DataPoint>>>,
    environment_data: Arc<Mutex<Vec<EnvironmentData>>>,
    channels: HashMap<String, ChannelData>,
    serial_thread: Option<thread::JoinHandle<()>>,
    stop_signal: Arc<Mutex<bool>>,
    show_graph: bool,
    start_time: f64,
    csv_filename: String,
    data_format: DataFormat,
    theme: Theme,
    show_stats: bool,
    graph_height: f32,
    connection_time: f64,
    total_bytes_received: usize,
    show_welcome: bool,
    show_environment_graphs: bool,
    serial_port: Option<Box<dyn std::io::Write + Send>>,
    current_cycle_status: Arc<Mutex<String>>,
    current_valve_state: Arc<Mutex<(u8, u8, u8, u8)>>,
    pending_sensor_map: Arc<Mutex<HashMap<String, f64>>>,
}

#[derive(PartialEq, Clone)]
enum DataFormat {
    SingleValue,
    CommaSeparated,
    JsonLike,
    Esp32,
}

#[derive(PartialEq, Clone)]
enum Theme {
    Dark,
    Light,
    Blue,
}

impl Default for SerialMonitorApp {
    fn default() -> Self {
        Self {
            available_ports: Vec::new(),
            selected_port: String::new(),
            baud_rate: "9600".to_string(),
            is_connected: false,
            received_data: Arc::new(Mutex::new(Vec::new())),
            plot_data: Arc::new(Mutex::new(Vec::new())),
            environment_data: Arc::new(Mutex::new(Vec::new())),
            channels: HashMap::new(),
            serial_thread: None,
            stop_signal: Arc::new(Mutex::new(false)),
            show_graph: true,
            start_time: 0.0,
            csv_filename: format!("enose_data_{}", Local::now().format("%Y%m%d_%H%M%S")),
            data_format: DataFormat::Esp32,
            theme: Theme::Dark,
            show_stats: true,
            graph_height: 250.0,
            connection_time: 0.0,
            total_bytes_received: 0,
            show_welcome: true,
            show_environment_graphs: true,
            serial_port: None,
            current_cycle_status: Arc::new(Mutex::new("---".to_string())),
            current_valve_state: Arc::new(Mutex::new((0, 0, 0, 0))),
            pending_sensor_map: Arc::new(Mutex::new(HashMap::new())),
        }
    }
}

fn parse_sensors_into(sensors_content: &str, map: &mut HashMap<String, f64>) {
    for entry in sensors_content.split(", ") {
        if let Some((name_part, val_part)) = entry.split_once(": ") {
            let name_trimmed = name_part.trim();
            let (key, raw_val) = if let Some(v) = val_part.strip_prefix("val=") {
                (name_trimmed.to_string(), v.trim())
            } else if let Some(v) = val_part.strip_prefix("raw=") {
                let key = name_trimmed.replace(" (", "_").replace(')', "");
                (key, v.trim())
            } else {
                continue;
            };
            if let Ok(value) = raw_val.parse::<f64>() {
                map.insert(key, value);
            }
        }
    }
}

impl SerialMonitorApp {
    fn apply_theme(&self, ctx: &egui::Context) {
        let visuals = match self.theme {
            Theme::Dark => {
                let mut v = egui::Visuals::dark();
                v.window_fill = Color32::from_rgb(20, 20, 25);
                v.panel_fill = Color32::from_rgb(25, 25, 30);
                v.extreme_bg_color = Color32::from_rgb(15, 15, 20);
                v
            }
            Theme::Light => {
                let mut v = egui::Visuals::light();
                v.window_fill = Color32::from_rgb(245, 245, 250);
                v.panel_fill = Color32::from_rgb(255, 255, 255);
                v
            }
            Theme::Blue => {
                let mut v = egui::Visuals::dark();
                v.window_fill = Color32::from_rgb(15, 20, 35);
                v.panel_fill = Color32::from_rgb(20, 25, 40);
                v.extreme_bg_color = Color32::from_rgb(10, 15, 30);
                v.widgets.noninteractive.bg_fill = Color32::from_rgb(30, 40, 60);
                v
            }
        };
        
        ctx.set_visuals(visuals);
    }

    fn scan_ports(&mut self) {
        self.available_ports.clear();
        if let Ok(ports) = serialport::available_ports() {
            for port in ports {
                self.available_ports.push(port.port_name);
            }
        }
        // Fallback: manually check common Linux serial ports not listed by serialport crate
        for pattern in &["/dev/ttyACM", "/dev/ttyUSB", "/dev/ttyS"] {
            for i in 0..10 {
                let port = format!("{}{}", pattern, i);
                if std::path::Path::new(&port).exists() && !self.available_ports.contains(&port) {
                    self.available_ports.push(port);
                }
            }
        }
        self.available_ports.sort();
        if !self.available_ports.is_empty() && self.selected_port.is_empty() {
            self.selected_port = self.available_ports[0].clone();
        }
    }

    fn get_channel_color(index: usize) -> Color32 {
        let colors = [
            Color32::from_rgb(52, 152, 219),   // Blue
            Color32::from_rgb(231, 76, 60),    // Red
            Color32::from_rgb(46, 204, 113),   // Green
            Color32::from_rgb(241, 196, 15),   // Yellow
            Color32::from_rgb(155, 89, 182),   // Purple
            Color32::from_rgb(26, 188, 156),   // Turquoise
            Color32::from_rgb(230, 126, 34),   // Orange
            Color32::from_rgb(236, 240, 241),  // Silver
        ];
        colors[index % colors.len()]
    }

    fn connect(&mut self) {
        let port_name = self.selected_port.clone();
        let baud_rate: u32 = self.baud_rate.parse().unwrap_or(9600);
        let received_data = Arc::clone(&self.received_data);
        let plot_data = Arc::clone(&self.plot_data);
        let environment_data = Arc::clone(&self.environment_data);
        let stop_signal = Arc::clone(&self.stop_signal);
        let data_format = self.data_format.clone();
        let current_cycle_status = Arc::clone(&self.current_cycle_status);
        let current_valve_state = Arc::clone(&self.current_valve_state);
        let pending_sensor_map = Arc::clone(&self.pending_sensor_map);
        
        *stop_signal.lock().unwrap() = false;
        self.start_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        self.connection_time = self.start_time;

        let start_time = self.start_time;

        let handle = thread::spawn(move || {
            match serialport::new(&port_name, baud_rate)
                .timeout(Duration::from_millis(100))
                .open()
            {
                Ok(mut port) => {
                    let mut buffer = vec![0u8; 1024];
                    let mut line_buffer = String::new();

                    loop {
                        if *stop_signal.lock().unwrap() {
                            break;
                        }

                        match port.read(&mut buffer) {
                            Ok(bytes_read) => {
                                let data = String::from_utf8_lossy(&buffer[..bytes_read]);
                                line_buffer.push_str(&data);

                                while let Some(pos) = line_buffer.find('\n') {
                                    let line = line_buffer[..pos].trim().to_string();
                                    if !line.is_empty() {
                                        let current_time = std::time::SystemTime::now()
                                            .duration_since(std::time::UNIX_EPOCH)
                                            .unwrap()
                                            .as_secs_f64() - start_time;

                                        let mut data = received_data.lock().unwrap();
                                        data.push(line.clone());
                                        if data.len() > 1000 {
                                            data.remove(0);
                                        }

                                        let values = match data_format {
                                            DataFormat::SingleValue => {
                                                let mut map = HashMap::new();
                                                if let Ok(value) = line.trim().parse::<f64>() {
                                                    map.insert("sensor".to_string(), value);
                                                }
                                                map
                                            }
                                            DataFormat::CommaSeparated => {
                                                let mut map = HashMap::new();
                                                for part in line.split(',') {
                                                    if let Some((key, val)) = part.split_once(':') {
                                                        if let Ok(value) = val.trim().parse::<f64>() {
                                                            map.insert(key.trim().to_string(), value);
                                                        }
                                                    }
                                                }
                                                map
                                            }
                                            DataFormat::JsonLike => {
                                                let mut map = HashMap::new();
                                                let cleaned = line.trim().trim_start_matches('{').trim_end_matches('}');
                                                for part in cleaned.split(',') {
                                                    if let Some((key, val)) = part.split_once(':') {
                                                        if let Ok(value) = val.trim().parse::<f64>() {
                                                            map.insert(key.trim().to_string(), value);
                                                        }
                                                    }
                                                }
                                                map
                                            }
                                            DataFormat::Esp32 => {
                                                let mut map = HashMap::new();
                                                if let Some(idx) = line.find("esp32_enose: ") {
                                                    let content = &line[idx + "esp32_enose: ".len()..];

                                                    if content.starts_with("STATE:") {
                                                        // Line 1: STATE:...,V_IN=... | SENSORS:H2S (D4): raw=..., Smoke (D7): raw=...
                                                        let parts: Vec<&str> = content.splitn(2, " | ").collect();

                                                        // Parse STATE section
                                                        if let Some(state_content) = parts[0].strip_prefix("STATE:") {
                                                            let mut status = String::from("---");
                                                            let mut v_in: u8 = 0;
                                                            let mut p_in: u8 = 0;
                                                            let mut v_out: u8 = 0;
                                                            let mut p_out: u8 = 0;
                                                            for kv in state_content.split(',') {
                                                                if let Some((k, v)) = kv.split_once('=') {
                                                                    match k.trim() {
                                                                        "STATUS" => status = v.trim().to_string(),
                                                                        "V_IN"   => v_in  = v.trim().parse().unwrap_or(0),
                                                                        "P_IN"   => p_in  = v.trim().parse().unwrap_or(0),
                                                                        "V_OUT"  => v_out = v.trim().parse().unwrap_or(0),
                                                                        "P_OUT"  => p_out = v.trim().parse().unwrap_or(0),
                                                                        _ => {}
                                                                    }
                                                                }
                                                            }
                                                            *current_cycle_status.lock().unwrap() = status;
                                                            *current_valve_state.lock().unwrap() = (v_in, p_in, v_out, p_out);
                                                        }

                                                        // Parse partial SENSORS from line 1, store in pending
                                                        let mut pending = pending_sensor_map.lock().unwrap();
                                                        pending.clear();
                                                        if let Some(sensors_part) = parts.get(1) {
                                                            if let Some(sensors_content) = sensors_part.strip_prefix("SENSORS:") {
                                                                parse_sensors_into(sensors_content, &mut pending);
                                                            }
                                                        }
                                                        // No DataPoint emitted yet — wait for line 2
                                                    } else if content.starts_with("SENSORS:") {
                                                        // Line 2: SENSORS:H2S (D15): raw=..., Temp: val=...
                                                        if let Some(sensors_content) = content.strip_prefix("SENSORS:") {
                                                            // Merge pending (line 1 sensors) into map
                                                            {
                                                                let pending = pending_sensor_map.lock().unwrap();
                                                                map.extend(pending.clone());
                                                            }
                                                            parse_sensors_into(sensors_content, &mut map);

                                                            // Update environment data
                                                            let sensor_temp = map.get("Temp").copied().unwrap_or(0.0);
                                                            let sensor_humid = map.get("Hum").copied().unwrap_or(0.0);
                                                            if sensor_temp > 0.0 || sensor_humid > 0.0 {
                                                                let mut env_data = environment_data.lock().unwrap();
                                                                env_data.push(EnvironmentData {
                                                                    timestamp: current_time,
                                                                    sample_temp: 0.0,
                                                                    sample_humid: 0.0,
                                                                    chamber_temp: 0.0,
                                                                    chamber_humid: 0.0,
                                                                });
                                                                if env_data.len() > 500 {
                                                                    env_data.remove(0);
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                                map
                                            }
                                        };

                                        if !values.is_empty() {
                                            // Store main sensor data
                                            let mut plot = plot_data.lock().unwrap();
                                            plot.push(DataPoint {
                                                timestamp: current_time,
                                                values: values.clone(),
                                                raw_text: line.clone(),
                                            });
                                            if plot.len() > 500 {
                                                plot.remove(0);
                                            }

                                            // Extract environment data if available
                                            let sample_temp = values.get("sample_temp").copied().unwrap_or(0.0);
                                            let sample_humid = values.get("sample_humid").copied().unwrap_or(0.0);
                                            let chamber_temp = values.get("chamber_temp").copied().unwrap_or(0.0);
                                            let chamber_humid = values.get("chamber_humid").copied().unwrap_or(0.0);

                                            if sample_temp > 0.0 || sample_humid > 0.0 || chamber_temp > 0.0 || chamber_humid > 0.0 {
                                                let mut env_data = environment_data.lock().unwrap();
                                                env_data.push(EnvironmentData {
                                                    timestamp: current_time,
                                                    sample_temp,
                                                    sample_humid,
                                                    chamber_temp,
                                                    chamber_humid,
                                                });
                                                if env_data.len() > 500 {
                                                    env_data.remove(0);
                                                }
                                            }
                                        }
                                    }
                                    line_buffer = line_buffer[pos + 1..].to_string();
                                }
                            }
                            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                                thread::sleep(Duration::from_millis(10));
                            }
                            Err(e) => {
                                let mut data = received_data.lock().unwrap();
                                data.push(format!("❌ Error: {}", e));
                                break;
                            }
                        }
                    }
                }
                Err(e) => {
                    let mut data = received_data.lock().unwrap();
                    data.push(format!("❌ Gagal membuka port: {}", e));
                }
            }
        });

        self.serial_thread = Some(handle);
        self.is_connected = true;
        self.show_welcome = false;
    }

    fn disconnect(&mut self) {
        *self.stop_signal.lock().unwrap() = true;
        if let Some(handle) = self.serial_thread.take() {
            let _ = handle.join();
        }
        self.serial_port = None;
        self.is_connected = false;
    }

    #[allow(dead_code)]
    fn send_command(&self, command: &str) {
        let cmd = format!("{}\n", command);
        if let Ok(port) = serialport::new(&self.selected_port, self.baud_rate.parse().unwrap_or(9600))
            .timeout(Duration::from_millis(100))
            .open()
        {
            let mut port = port;
            let _ = port.write_all(cmd.as_bytes());
            self.received_data.lock().unwrap().push(format!("➤ Command sent: {}", command));
        }
    }

    fn clear_data(&mut self) {
        self.received_data.lock().unwrap().clear();
        self.plot_data.lock().unwrap().clear();
        self.environment_data.lock().unwrap().clear();
        self.channels.clear();
        self.total_bytes_received = 0;
    }

    fn update_channels(&mut self) {
        let plot_data = self.plot_data.lock().unwrap();
        
        let mut all_channels = std::collections::HashSet::new();
        for point in plot_data.iter() {
            for key in point.values.keys() {
                // Skip environment data from main channels
                if !key.contains("temp") && !key.contains("humid") && !key.contains("sample_") && !key.contains("chamber_")
                    && !matches!(key.as_str(), "STATUS" | "V_IN" | "P_IN" | "V_OUT" | "P_OUT"
                        | "Temp" | "Hum" | "Press" | "Gas_Ohm" | "Gas_Volt")
                {
                    all_channels.insert(key.clone());
                }
            }
        }

        let mut channel_index = self.channels.len();
        for channel_name in all_channels {
            if !self.channels.contains_key(&channel_name) {
                self.channels.insert(
                    channel_name.clone(),
                    ChannelData {
                        points: Vec::new(),
                        color: Self::get_channel_color(channel_index),
                        visible: true,
                        min_value: f64::INFINITY,
                        max_value: f64::NEG_INFINITY,
                        avg_value: 0.0,
                    },
                );
                channel_index += 1;
            }
        }

        for (channel_name, channel_data) in self.channels.iter_mut() {
            channel_data.points.clear();
            let mut sum = 0.0;
            let mut count = 0;
            
            for point in plot_data.iter() {
                if let Some(&value) = point.values.get(channel_name) {
                    channel_data.points.push((point.timestamp, value));
                    channel_data.min_value = channel_data.min_value.min(value);
                    channel_data.max_value = channel_data.max_value.max(value);
                    sum += value;
                    count += 1;
                }
            }
            
            if count > 0 {
                channel_data.avg_value = sum / count as f64;
            }
        }
    }

    fn save_to_csv(&self) -> Result<String, String> {
        let plot_data = self.plot_data.lock().unwrap();
        
        if plot_data.is_empty() {
            return Err("Tidak ada data untuk disimpan".to_string());
        }

        let filename = if self.csv_filename.ends_with(".csv") {
            self.csv_filename.clone()
        } else {
            format!("{}.csv", self.csv_filename)
        };

        match File::create(&filename) {
            Ok(mut file) => {
                let mut all_channels = std::collections::HashSet::new();
                for point in plot_data.iter() {
                    for key in point.values.keys() {
                        all_channels.insert(key.clone());
                    }
                }
                let mut channel_names: Vec<String> = all_channels.into_iter().collect();
                channel_names.sort();

                write!(file, "Timestamp (s)").map_err(|e| e.to_string())?;
                for name in &channel_names {
                    write!(file, ",{}", name).map_err(|e| e.to_string())?;
                }
                writeln!(file, ",Raw Text").map_err(|e| e.to_string())?;

                for point in plot_data.iter() {
                    write!(file, "{:.3}", point.timestamp).map_err(|e| e.to_string())?;
                    for name in &channel_names {
                        if let Some(&value) = point.values.get(name) {
                            write!(file, ",{:.3}", value).map_err(|e| e.to_string())?;
                        } else {
                            write!(file, ",").map_err(|e| e.to_string())?;
                        }
                    }
                    writeln!(file, ",\"{}\"", point.raw_text).map_err(|e| e.to_string())?;
                }

                Ok(format!("Data berhasil disimpan ke {}", filename))
            }
            Err(e) => Err(format!("Gagal membuat file: {}", e)),
        }
    }

    fn draw_welcome_screen(&self, ui: &mut egui::Ui) {
        ui.vertical_centered(|ui| {
            ui.add_space(50.0);
            
            ui.label(RichText::new("🔬").size(80.0));
            ui.add_space(20.0);
            
            ui.label(RichText::new("MEMS Gas Array Sensors")
                .size(32.0)
                .strong());
            ui.label(RichText::new("E-Nose Chamber Environment Monitor")
                .size(18.0)
                .color(Color32::GRAY));
            
            ui.add_space(30.0);
            
            ui.label("Monitoring Parameters:");
            ui.add_space(10.0);
            
            ui.horizontal(|ui| {
                ui.add_space(50.0);
                ui.vertical(|ui| {
                    ui.label("🌡️ Sample Chamber Temperature & Humidity");
                    ui.label("🌡️ Sensor Chamber Temperature & Humidity");
                    ui.label("📊 Multi-channel gas sensor data");
                    ui.label("💾 CSV export dengan timestamp");
                    ui.label("📈 Real-time graphing & analytics");
                    ui.label("🎨 Multiple themes support");
                });
            });
            
            ui.add_space(30.0);
            ui.separator();
            ui.add_space(20.0);
            
            ui.label(RichText::new("👈 Select port and click Connect to start monitoring")
                .size(16.0)
                .color(Color32::from_rgb(52, 152, 219)));
            
            ui.add_space(10.0);
            ui.label(RichText::new("Data Format Example:")
                .size(14.0)
                .strong());
            ui.label(RichText::new("I (20552) esp32_enose: STATE:STATUS=IDLE,V_IN=1,P_IN=0,V_OUT=1,P_OUT=0 | SENSORS:H2S (D4): raw=363, Temp: val=25.67, ...")
                .size(11.0)
                .color(Color32::GRAY)
                .monospace());
        });
    }
}

impl eframe::App for SerialMonitorApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.apply_theme(ctx);
        ctx.request_repaint();
        self.update_channels();

        // Modern Top Bar
        egui::TopBottomPanel::top("top_panel")
            .exact_height(60.0)
            .show(ctx, |ui| {
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    ui.add_space(10.0);
                    
                    ui.label(RichText::new("🔬").size(24.0));
                    ui.label(RichText::new("E-Nose Chamber Monitor")
                        .size(20.0)
                        .strong());
                    
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.add_space(10.0);
                        
                        egui::ComboBox::from_id_source("theme_selector")
                            .selected_text(match self.theme {
                                Theme::Dark => "🌙 Dark",
                                Theme::Light => "☀️ Light",
                                Theme::Blue => "🌊 Blue",
                            })
                            .width(100.0)
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.theme, Theme::Dark, "🌙 Dark");
                                ui.selectable_value(&mut self.theme, Theme::Light, "☀️ Light");
                                ui.selectable_value(&mut self.theme, Theme::Blue, "🌊 Blue");
                            });
                        
                        ui.separator();
                        
                        let (status_text, status_color) = if self.is_connected {
                            ("CONNECTED", Color32::from_rgb(46, 204, 113))
                        } else {
                            ("DISCONNECTED", Color32::from_rgb(231, 76, 60))
                        };
                        
                        ui.label(RichText::new("●")
                            .size(16.0)
                            .color(status_color));
                        ui.label(RichText::new(status_text)
                            .size(14.0)
                            .strong()
                            .color(status_color));
                    });
                });
                ui.add_space(10.0);
            });

        // Sidebar
        egui::SidePanel::left("control_panel")
            .min_width(300.0)
            .max_width(320.0)
            .resizable(false)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false; 2])
                    .show(ui, |ui| {
                    ui.add_space(15.0);
                    
                    // Connection Panel
                    egui::Frame::group(ui.style())
                        .fill(if matches!(self.theme, Theme::Dark | Theme::Blue) {
                            Color32::from_rgb(30, 35, 45)
                        } else {
                            Color32::from_rgb(248, 249, 250)
                        })
                        .stroke(Stroke::new(1.0, Color32::from_rgb(52, 152, 219)))
                        .inner_margin(15.0)
                        .show(ui, |ui| {
                            ui.label(RichText::new("⚙ Connection Settings")
                                .size(16.0)
                                .strong()
                                .color(Color32::from_rgb(52, 152, 219)));
                            ui.add_space(10.0);
                            
                            ui.label(RichText::new("Serial Port:").strong());
                            ui.add_space(3.0);
                            egui::ComboBox::from_id_source("port")
                                .selected_text(if self.selected_port.is_empty() { 
                                    "Select port..." 
                                } else { 
                                    &self.selected_port 
                                })
                                .width(ui.available_width())
                                .show_ui(ui, |ui| {
                                    for port in &self.available_ports.clone() {
                                        ui.selectable_value(&mut self.selected_port, port.clone(), port);
                                    }
                                });
                            
                            ui.add_space(8.0);
                            ui.label(RichText::new("Baud Rate:").strong());
                            ui.add_space(3.0);
                            egui::ComboBox::from_id_source("baud")
                                .selected_text(&self.baud_rate)
                                .width(ui.available_width())
                                .show_ui(ui, |ui| {
                                    for rate in &["9600", "19200", "38400", "57600", "115200"] {
                                        ui.selectable_value(&mut self.baud_rate, rate.to_string(), *rate);
                                    }
                                });
                        });

                    ui.add_space(12.0);

                    // Data Format Panel
                    egui::Frame::group(ui.style())
                        .fill(if matches!(self.theme, Theme::Dark | Theme::Blue) {
                            Color32::from_rgb(30, 35, 45)
                        } else {
                            Color32::from_rgb(248, 249, 250)
                        })
                        .inner_margin(15.0)
                        .show(ui, |ui| {
                            ui.label(RichText::new("📊 Data Format").size(16.0).strong());
                            ui.add_space(8.0);
                            
                            ui.radio_value(&mut self.data_format, DataFormat::CommaSeparated, 
                                RichText::new("CSV Format (Recommended)").size(13.0));
                            ui.label(RichText::new("  key:value,key:value").size(11.0).color(Color32::GRAY));
                            ui.add_space(5.0);
                            
                            ui.radio_value(&mut self.data_format, DataFormat::JsonLike, 
                                RichText::new("JSON-like").size(13.0));
                            ui.label(RichText::new("  {key:value,key:value}").size(11.0).color(Color32::GRAY));
                            ui.add_space(5.0);
                            
                            ui.radio_value(&mut self.data_format, DataFormat::SingleValue, 
                                RichText::new("Single Value").size(13.0));
                            ui.label(RichText::new("  123.45").size(11.0).color(Color32::GRAY));
                            ui.add_space(5.0);

                            ui.radio_value(&mut self.data_format, DataFormat::Esp32,
                                RichText::new("ESP32 E-Nose").size(13.0));
                            ui.label(RichText::new("  I (...) esp32_enose: STATE:... | SENSORS:...").size(11.0).color(Color32::GRAY));
                        });

                    ui.add_space(12.0);

                    // Control Buttons
                    ui.vertical(|ui| {
                        if ui.add_sized(
                            [ui.available_width(), 35.0],
                            egui::Button::new(RichText::new("🔍 Scan Ports").size(14.0))
                        ).clicked() {
                            self.scan_ports();
                        }
                        
                        ui.add_space(8.0);
                        
                        if !self.is_connected {
                            if ui.add_sized(
                                [ui.available_width(), 40.0],
                                egui::Button::new(RichText::new("🔌 Connect").size(15.0).strong())
                                    .fill(Color32::from_rgb(46, 204, 113))
                            ).clicked() && !self.selected_port.is_empty() {
                                self.connect();
                            }
                        } else {
                            if ui.add_sized(
                                [ui.available_width(), 40.0],
                                egui::Button::new(RichText::new("⏹ Disconnect").size(15.0).strong())
                                    .fill(Color32::from_rgb(231, 76, 60))
                            ).clicked() {
                                self.disconnect();
                            }
                        }
                        
                        ui.add_space(8.0);
                        
                        if ui.add_sized(
                            [ui.available_width(), 35.0],
                            egui::Button::new(RichText::new("🗑 Clear All Data").size(14.0))
                        ).clicked() {
                            self.clear_data();
                        }
                    });

                    ui.add_space(12.0);

                    // CSV Export Panel
                    egui::Frame::group(ui.style())
                        .fill(Color32::from_rgb(52, 152, 219).linear_multiply(0.1))
                        .inner_margin(15.0)
                        .show(ui, |ui| {
                            ui.label(RichText::new("💾 Export Data").size(16.0).strong());
                            ui.add_space(8.0);
                            
                            ui.label(RichText::new("Filename:").strong());
                            ui.add_space(3.0);
                            ui.text_edit_singleline(&mut self.csv_filename);
                            ui.label(RichText::new("(.csv akan ditambahkan otomatis)")
                                .size(10.0)
                                .color(Color32::GRAY));
                            
                            ui.add_space(8.0);
                            
                            if ui.add_sized(
                                [ui.available_width(), 35.0],
                                egui::Button::new(RichText::new("💾 Export to CSV").size(14.0))
                                    .fill(Color32::from_rgb(52, 152, 219))
                            ).clicked() {
                                match self.save_to_csv() {
                                    Ok(msg) => {
                                        self.received_data.lock().unwrap().push(format!("✅ {}", msg));
                                    }
                                    Err(msg) => {
                                        self.received_data.lock().unwrap().push(format!("❌ {}", msg));
                                    }
                                }
                            }
                        });

                    ui.add_space(12.0);

                    // Display Options
                    egui::Frame::group(ui.style())
                        .inner_margin(15.0)
                        .show(ui, |ui| {
                            ui.label(RichText::new("👁 Display Options").size(16.0).strong());
                            ui.add_space(8.0);
                            
                            ui.checkbox(&mut self.show_graph, RichText::new("📊 Sensor Graphs").size(13.0));
                            ui.checkbox(&mut self.show_environment_graphs, RichText::new("🌡️ Environment Graphs").size(13.0));
                            ui.checkbox(&mut self.show_stats, RichText::new("📈 Statistics").size(13.0));
                            
                            ui.add_space(8.0);
                            ui.label(RichText::new("Graph Height:").strong());
                            ui.add(egui::Slider::new(&mut self.graph_height, 150.0..=400.0).suffix(" px"));
                        });

                    ui.add_space(12.0);

                    // ESP32 Cycle Status Panel
                    egui::Frame::group(ui.style())
                        .fill(if matches!(self.theme, Theme::Dark | Theme::Blue) {
                            Color32::from_rgb(30, 35, 45)
                        } else {
                            Color32::from_rgb(248, 249, 250)
                        })
                        .stroke(Stroke::new(1.0, Color32::from_rgb(155, 89, 182)))
                        .inner_margin(15.0)
                        .show(ui, |ui| {
                            ui.label(RichText::new("⚡ Cycle Status").size(16.0).strong().color(Color32::from_rgb(155, 89, 182)));
                            ui.add_space(10.0);

                            let status = self.current_cycle_status.lock().unwrap().clone();
                            let status_color = match status.as_str() {
                                "IDLE"      => Color32::from_rgb(46, 204, 113),
                                "INJECTING" => Color32::from_rgb(52, 152, 219),
                                "SENSING"   => Color32::from_rgb(241, 196, 15),
                                "PURGING"   => Color32::from_rgb(231, 76, 60),
                                _           => Color32::GRAY,
                            };
                            ui.vertical_centered(|ui| {
                                ui.label(RichText::new(&status)
                                    .size(28.0)
                                    .strong()
                                    .color(status_color));
                            });

                            ui.add_space(10.0);

                            let (v_in, p_in, v_out, p_out) = *self.current_valve_state.lock().unwrap();
                            let val_color = |v: u8| if v == 1 { Color32::from_rgb(46, 204, 113) } else { Color32::from_rgb(231, 76, 60) };
                            ui.columns(2, |cols| {
                                cols[0].label(RichText::new(format!("V_IN: {}", v_in)).size(12.0).color(val_color(v_in)));
                                cols[0].label(RichText::new(format!("P_IN: {}", p_in)).size(12.0).color(val_color(p_in)));
                                cols[1].label(RichText::new(format!("V_OUT: {}", v_out)).size(12.0).color(val_color(v_out)));
                                cols[1].label(RichText::new(format!("P_OUT: {}", p_out)).size(12.0).color(val_color(p_out)));
                            });
                        });

                    ui.add_space(12.0);

                    // Channels
                    if !self.channels.is_empty() {
                        egui::Frame::group(ui.style())
                            .inner_margin(15.0)
                            .show(ui, |ui| {
                                ui.label(RichText::new("📡 Gas Sensors").size(16.0).strong());
                                ui.add_space(8.0);
                                
                                for (name, channel) in self.channels.iter_mut() {
                                    ui.horizontal(|ui| {
                                        ui.checkbox(&mut channel.visible, "");
                                        ui.label(RichText::new("●").size(16.0).color(channel.color));
                                        ui.label(RichText::new(name).strong());
                                        
                                        if channel.visible && !channel.points.is_empty() {
                                            let last_val = channel.points.last().unwrap().1;
                                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                ui.label(RichText::new(format!("{:.2}", last_val))
                                                    .size(12.0)
                                                    .color(Color32::GRAY));
                                            });
                                        }
                                    });
                                }
                            });
                        
                        ui.add_space(12.0);
                    }

                    // Statistics Panel
                    if self.show_stats {
                        egui::Frame::group(ui.style())
                            .fill(Color32::from_rgb(26, 188, 156).linear_multiply(0.1))
                            .inner_margin(15.0)
                            .show(ui, |ui| {
                                ui.label(RichText::new("📊 Statistics").size(16.0).strong());
                                ui.add_space(8.0);
                                
                                let data_len = self.received_data.lock().unwrap().len();
                                let plot_len = self.plot_data.lock().unwrap().len();
                                let env_len = self.environment_data.lock().unwrap().len();
                                
                                ui.horizontal(|ui| {
                                    ui.label("📝 Messages:");
                                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                        ui.label(RichText::new(data_len.to_string()).strong());
                                    });
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("📊 Sensor Points:");
                                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                        ui.label(RichText::new(plot_len.to_string()).strong());
                                    });
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("🌡️ Environment Points:");
                                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                        ui.label(RichText::new(env_len.to_string()).strong());
                                    });
                                });
                                
                                ui.horizontal(|ui| {
                                    ui.label("📡 Active Sensors:");
                                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                        ui.label(RichText::new(self.channels.len().to_string()).strong());
                                    });
                                });
                                
                                if self.is_connected {
                                    let uptime = std::time::SystemTime::now()
                                        .duration_since(std::time::UNIX_EPOCH)
                                        .unwrap()
                                        .as_secs_f64() - self.connection_time;
                                    
                                    ui.horizontal(|ui| {
                                        ui.label("⏱ Uptime:");
                                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                            ui.label(RichText::new(format!("{:.0}s", uptime)).strong());
                                        });
                                    });
                                }
                            });
                    }

                    ui.add_space(15.0);
                });
            });

        // Main Content
        egui::CentralPanel::default().show(ctx, |ui| {
            if self.show_welcome && !self.is_connected {
                self.draw_welcome_screen(ui);
            } else {
                egui::ScrollArea::vertical()
                    .auto_shrink([false; 2])
                    .show(ui, |ui| {
                        ui.add_space(10.0);
                        
                        // Environment Graphs Section
                        if self.show_environment_graphs {
                            let env_data = self.environment_data.lock().unwrap();
                            
                            if !env_data.is_empty() {
                                ui.heading(RichText::new("🌡️ Chamber Environment Monitoring").size(20.0).strong());
                                ui.add_space(10.0);
                                
                                ui.columns(2, |columns| {
                                    // Column 1: Gas Chamber (BME688)
                                    columns[0].group(|ui| {
                                        ui.label(RichText::new("💨 Gas Chamber (BME688)").size(16.0).strong().color(Color32::from_rgb(46, 204, 113)));
                                        ui.add_space(5.0);
                                        
                                        // Temperature
                                        ui.label(RichText::new("Temperature (°C)").strong());
                                        let temp_points: PlotPoints = env_data.iter()
                                            .map(|d| [d.timestamp, d.sample_temp])
                                            .collect();
                                        
                                        Plot::new("sample_temp_plot")
                                            .height(self.graph_height)
                                            .show_axes([true, true])
                                            .show_grid([true, true])
                                            .allow_zoom(true)
                                            .allow_drag(true)
                                            .show(ui, |plot_ui| {
                                                plot_ui.line(
                                                    Line::new(temp_points)
                                                        .name("Temperature")
                                                        .color(Color32::from_rgb(231, 76, 60))
                                                        .width(2.5)
                                                );
                                            });
                                        
                                        ui.add_space(10.0);
                                        
                                        // Humidity
                                        ui.label(RichText::new("Humidity (%)").strong());
                                        let humid_points: PlotPoints = env_data.iter()
                                            .map(|d| [d.timestamp, d.sample_humid])
                                            .collect();
                                        
                                        Plot::new("sample_humid_plot")
                                            .height(self.graph_height)
                                            .show_axes([true, true])
                                            .show_grid([true, true])
                                            .allow_zoom(true)
                                            .allow_drag(true)
                                            .show(ui, |plot_ui| {
                                                plot_ui.line(
                                                    Line::new(humid_points)
                                                        .name("Humidity")
                                                        .color(Color32::from_rgb(52, 152, 219))
                                                        .width(2.5)
                                                );
                                            });
                                    });
                                    
                                    // Column 2: Object Chamber (AHT25)
                                    columns[1].group(|ui| {
                                        ui.label(RichText::new("🌡️ Object Chamber (AHT25)").size(16.0).strong().color(Color32::from_rgb(155, 89, 182)));
                                        ui.add_space(5.0);
                                        
                                        // Temperature
                                        ui.label(RichText::new("Temperature (°C)").strong());
                                        let temp_points: PlotPoints = env_data.iter()
                                            .map(|d| [d.timestamp, d.chamber_temp])
                                            .collect();
                                        
                                        Plot::new("chamber_temp_plot")
                                            .height(self.graph_height)
                                            .show_axes([true, true])
                                            .show_grid([true, true])
                                            .allow_zoom(true)
                                            .allow_drag(true)
                                            .show(ui, |plot_ui| {
                                                plot_ui.line(
                                                    Line::new(temp_points)
                                                        .name("Temperature")
                                                        .color(Color32::from_rgb(241, 196, 15))
                                                        .width(2.5)
                                                );
                                            });
                                        
                                        ui.add_space(10.0);
                                        
                                        // Humidity
                                        ui.label(RichText::new("Humidity (%)").strong());
                                        let humid_points: PlotPoints = env_data.iter()
                                            .map(|d| [d.timestamp, d.chamber_humid])
                                            .collect();
                                        
                                        Plot::new("chamber_humid_plot")
                                            .height(self.graph_height)
                                            .show_axes([true, true])
                                            .show_grid([true, true])
                                            .allow_zoom(true)
                                            .allow_drag(true)
                                            .show(ui, |plot_ui| {
                                                plot_ui.line(
                                                    Line::new(humid_points)
                                                        .name("Humidity")
                                                        .color(Color32::from_rgb(52, 152, 219))
                                                        .width(2.5)
                                                );
                                            });
                                    });
                                });
                                
                                ui.add_space(20.0);
                                ui.separator();
                                ui.add_space(15.0);
                            }
                        }
                        
                        // Gas Sensor Graphs
                        if self.show_graph && !self.channels.is_empty() {
                            egui::Frame::group(ui.style())
                                .inner_margin(10.0)
                                .show(ui, |ui| {
                                    ui.label(RichText::new("📊 Gas Sensor Array - Real-Time Data").size(18.0).strong());
                                    ui.add_space(5.0);
                                    
                                    Plot::new("main_sensor_plot")
                                        .height(self.graph_height)
                                        .legend(Legend::default()
                                            .position(Corner::RightTop)
                                            .background_alpha(0.95))
                                        .show_axes([true, true])
                                        .show_grid([true, true])
                                        .allow_zoom(true)
                                        .allow_drag(true)
                                        .allow_scroll(true)
                                        .show(ui, |plot_ui| {
                                            for (name, channel) in &self.channels {
                                                if channel.visible && !channel.points.is_empty() {
                                                    let points: PlotPoints = channel.points
                                                        .iter()
                                                        .map(|&(t, v)| [t, v])
                                                        .collect();
                                                    
                                                    plot_ui.line(
                                                        Line::new(points)
                                                            .name(name)
                                                            .color(channel.color)
                                                            .width(2.5)
                                                    );
                                                }
                                            }
                                        });
                                });
                        }
                        
                        ui.add_space(10.0);
                    });
            }
        });
    }
}

pub fn run() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1600.0, 900.0])
            .with_min_inner_size([1200.0, 700.0])
            .with_title("MEMS Gas Array Sensors: E-Nose Chamber Environment Monitor"),
        ..Default::default()
    };

    eframe::run_native(
        "E-Nose Chamber Monitor",
        options,
        Box::new(|_cc| {
            let mut app = SerialMonitorApp::default();
            app.scan_ports();
            Box::new(app)
        }),
    )
}