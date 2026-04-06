#include "control_step.h"
#include "sensor.h"

const int pin_bao_mat_line = 45; 
const unsigned long time_bao_mat_line_lau = 30000; 
const int pin_dung = 47;
const int speed = 1200;
const int slowSpeed = 12000;
const int startSpeed = 12000;

const int BU_ZONE_123 = 0;
const int BU_ZONE_456 = 400;
const int BU_ZONE_789 = 500;

// BỘ PID GỐC
const float Kp_123 = 50.0;  const float Kd_123 = 400.0;
const float Kp_456 = 150.0; const float Kd_456 = 700.0;
const float Kp_789 = 140.0; const float Kd_789 = 700.0;
const float Ki = 0.01;

// NỘI SUY THỜI GIAN LÀM MƯỢT
const float TIME_VAO_CUA = 150.0; 
const float TIME_RA_CUA  = 500.0; 

// =======================================================================
// 🌟 THÔNG SỐ CÁC BỘ LỌC ẢO (KHÔNG LÀM HỎNG MẮT QUÉT) 🌟
// =======================================================================
// 1. Chống lắc tâm
const unsigned long TIME_DELAY_CASE_1 = 200; // Đợi 200ms mới bắt đầu bẻ lái nhẹ

// 2. Lướt ngã rẽ (Y-Junction)
const unsigned long TIME_LUOT_NGA_RE = 150; // "Bịt mắt" 150ms để lao thẳng qua ngã gộp
const int NGUONG_NGA_RE = 3;                // Bước nhảy từ 0,1 lên >=3 sẽ bị coi là ngã rẽ
// =======================================================================

const unsigned long thoi_gian_tang_toc = 9000;
const unsigned long thoi_gian_giam_toc = 6000;
const unsigned long time_dung_lay_hang = 20000;
const unsigned long THOI_GIAN_QUAY_MU = 4000;
const unsigned long time_hu_line = 300;
const int time_tang_toc_case14_lan_1 = 8000;

enum TrangThaiTram 
{
    CHAY_BINH_THUONG = 0,
    BAT_DAU_GIAM_TOC = 1,
    CHO_DUNG = 2,
    DANG_DUNG_LAY_HANG = 3,
    ROI_KHOI_TRAM = 4,
    QUAY_DAU = 5
};
TrangThaiTram trang_thai_14 = CHAY_BINH_THUONG;

unsigned long thoi_gian_bat_dau = 0, thoi_gian_bat_dau_giam = 0, t_dung = 0;
unsigned long t_chong_doi_tram = 0;
unsigned long last_ramp_time = 0;
unsigned long thoi_gian_bat_dau_quay = 0;
unsigned long thoi_gian_mat_line = 0;

bool dang_mat_line = false;
bool dang_dung_vi_vat_can = false;
bool dang_dung_vi_mat_line = false;
bool dang_tang_toc = true, dang_giam_toc = false;
bool is_special_code = false;

int speed_quay_dau = 4000;
float toc_do_khi_bat_dau_giam = 0, toc_do_khi_bat_dau_tang = 0;
float currentSpeedVal = startSpeed, smoothBaseSpeed = 0;
float smoothedError = 0, lastError = 0, integral = 0, lastPidOutput = 0;
int raw_sensor = 0;
int steering_error = 0;
int last_valid_sensor = 0;

void setup() 
{
    pinMode(pin_dung, OUTPUT); digitalWrite(pin_dung, HIGH);
    pinMode(pin_bao_mat_line, OUTPUT); digitalWrite(pin_bao_mat_line, HIGH);

    pinMode(PUL_TRAI, OUTPUT); pinMode(DIR_TRAI, OUTPUT); pinMode(ENA_TRAI, OUTPUT);
    pinMode(PUL_PHAI, OUTPUT); pinMode(DIR_PHAI, OUTPUT); pinMode(ENA_PHAI, OUTPUT);
    pinMode(T4, INPUT); pinMode(T3, INPUT); pinMode(T2, INPUT); pinMode(T1, INPUT);
    pinMode(TG, INPUT); pinMode(PG, INPUT);
    pinMode(P1, INPUT); pinMode(P2, INPUT); pinMode(P3, INPUT); pinMode(P4, INPUT);
    pinMode(cb1, INPUT_PULLUP);
    
    digitalWrite(ENA_TRAI, LOW); digitalWrite(ENA_PHAI, LOW);
    KhoiTaoThongSoDeBa();
    if (In_SenSor() == 14) 
    {
        trang_thai_14 = ROI_KHOI_TRAM; 
    }
}

void loop() 
{
    raw_sensor = In_SenSor();
    if (KiemTraAnToanGap()) return;
    DocVaLocCamBien();
    XuLyMayTrangThai();
    QuyetDinhXuatXung();
}

void KhoiTaoThongSoDeBa() 
{
    currentSpeedVal = (float)startSpeed; 
    smoothBaseSpeed = (float)startSpeed; 
    toc_do_khi_bat_dau_tang = (float)startSpeed; 
    thoi_gian_bat_dau = millis(); 
    last_ramp_time = millis(); 
    dang_tang_toc = true; dang_giam_toc = false;
    integral = 0; lastError = 0; smoothedError = 0; 
    last_valid_sensor = 0; lastPidOutput = 0;
}

bool KiemTraAnToanGap() 
{
    bool phai_dung_lai = false;
    if (digitalRead(cb1) == LOW) 
    {
        step_dc(false, false, HIGH, LOW, 0, 0);
        dang_dung_vi_vat_can = true; phai_dung_lai = true;
    } else { 
        if (dang_dung_vi_vat_can) 
        {
            KhoiTaoThongSoDeBa(); dang_dung_vi_vat_can = false;
        }
    }
    if (phai_dung_lai) return true; 
    
    if (raw_sensor == 13) 
    {
        if (!dang_mat_line) 
        {
            dang_mat_line = true; thoi_gian_mat_line = millis();
        }
        if (millis() - thoi_gian_mat_line >= time_hu_line) 
        {
            step_dc(false, false, HIGH, LOW, 0, 0);
            dang_dung_vi_mat_line = true; phai_dung_lai = true;
        } 
        if (millis() - thoi_gian_mat_line >= time_bao_mat_line_lau)
        {
            digitalWrite(pin_bao_mat_line, LOW);
        }
    } 
    else 
    { 
        dang_mat_line = false;
        digitalWrite(pin_bao_mat_line, HIGH);
        if (dang_dung_vi_mat_line) 
        {
            KhoiTaoThongSoDeBa(); dang_dung_vi_mat_line = false;
        }
    }
    return phai_dung_lai;
}

// =========================================================================
// TRẢ LẠI HÀM CẢM BIẾN NGUYÊN BẢN: Đọc 100% sự thật, không giấu diếm
// =========================================================================
void DocVaLocCamBien() 
{
    if (raw_sensor == 14 && trang_thai_14 == CHAY_BINH_THUONG) 
    {
        if (abs(last_valid_sensor) >= 6 || abs(smoothedError) > 5.5) 
        {
            raw_sensor = (last_valid_sensor > 0) ? 9 : -9;
        }
    }
    is_special_code = (raw_sensor == 12 || raw_sensor == 13 || raw_sensor == 14);
    if (!is_special_code) 
    {
        steering_error = raw_sensor;
        last_valid_sensor = raw_sensor; 
    } else {
        steering_error = 0; 
    }
}
// =========================================================================

void XuLyMayTrangThai() 
{
    if (raw_sensor == 12 && trang_thai_14 == CHAY_BINH_THUONG) 
    {
        trang_thai_14 = QUAY_DAU; thoi_gian_bat_dau_quay = millis(); 
    }
    if (trang_thai_14 == QUAY_DAU) 
    {
        if (millis() - thoi_gian_bat_dau_quay >= THOI_GIAN_QUAY_MU) 
        {
            if (raw_sensor == 0) 
            {
                trang_thai_14 = CHAY_BINH_THUONG; KhoiTaoThongSoDeBa(); 
            }
        }
        return; 
    }
    if (raw_sensor == 14) 
    {
        if (trang_thai_14 == CHAY_BINH_THUONG) 
        { 
            trang_thai_14 = BAT_DAU_GIAM_TOC; t_chong_doi_tram = millis(); 
            dang_tang_toc = false; dang_giam_toc = true;  
            thoi_gian_bat_dau_giam = millis(); toc_do_khi_bat_dau_giam = smoothBaseSpeed; 
        } else if (trang_thai_14 == CHO_DUNG) 
        { 
            if (millis() - t_chong_doi_tram > 500) 
            {
                trang_thai_14 = DANG_DUNG_LAY_HANG; t_dung = millis(); 
                digitalWrite(pin_dung, LOW); 
            }
        }
    }
    if (trang_thai_14 == BAT_DAU_GIAM_TOC && !is_special_code && (millis() - t_chong_doi_tram > 150)) 
    {
        trang_thai_14 = CHO_DUNG; 
    }
    if ((trang_thai_14 == BAT_DAU_GIAM_TOC || trang_thai_14 == CHO_DUNG) && (millis() - thoi_gian_bat_dau_giam > time_tang_toc_case14_lan_1)) 
    {
        trang_thai_14 = CHAY_BINH_THUONG; 
        dang_giam_toc = false; dang_tang_toc = true; 
        toc_do_khi_bat_dau_tang = smoothBaseSpeed; thoi_gian_bat_dau = millis();
    }
    if (trang_thai_14 == DANG_DUNG_LAY_HANG && (millis() - t_dung >= time_dung_lay_hang)) 
    {
        trang_thai_14 = ROI_KHOI_TRAM; digitalWrite(pin_dung, HIGH); KhoiTaoThongSoDeBa(); 
    }
    if (trang_thai_14 == ROI_KHOI_TRAM && !is_special_code) 
    {
        trang_thai_14 = CHAY_BINH_THUONG; 
    }
}

void QuyetDinhXuatXung() 
{
    if (trang_thai_14 == DANG_DUNG_LAY_HANG) 
    {
        step_dc(false, false, HIGH, LOW, 0, 0); last_ramp_time = millis(); return; 
    } 
    if (trang_thai_14 == QUAY_DAU) 
    {
        step_dc(true, true, LOW, LOW, speed_quay_dau, speed_quay_dau); last_ramp_time = millis(); return; 
    } 
    TinhToanBiendangTocDo();
    TinhToanVaXuatPID();
}

void TinhToanBiendangTocDo() 
{
    if (dang_tang_toc) 
    {
        unsigned long t_chay = millis() - thoi_gian_bat_dau;
        if (t_chay < thoi_gian_tang_toc) 
        {
            currentSpeedVal = toc_do_khi_bat_dau_tang - ((toc_do_khi_bat_dau_tang - speed) * ((float)t_chay / thoi_gian_tang_toc));
        } else { 
            currentSpeedVal = (float)speed; dang_tang_toc = false; 
        }
    } 
    else if (dang_giam_toc) 
    {
        unsigned long t_giam = millis() - thoi_gian_bat_dau_giam;
        if (t_giam < thoi_gian_giam_toc) {
            currentSpeedVal = toc_do_khi_bat_dau_giam + ((slowSpeed - toc_do_khi_bat_dau_giam) * ((float)t_giam / thoi_gian_giam_toc));
        } else { 
            currentSpeedVal = (float)slowSpeed; dang_giam_toc = false; 
        }
    }
    smoothBaseSpeed = (currentSpeedVal * 0.1) + (smoothBaseSpeed * 0.9);
}

void TinhToanVaXuatPID() 
{
    int baseSpeed = (int)smoothBaseSpeed;
    
    // =========================================================================
    // BẮT ĐẦU: BỘ LỌC ẢO (Xử lý lỗi trước khi đưa vào PID)
    // =========================================================================
    int rawAbsError = abs(steering_error);
    static int previous_rawAbsError = 0;
    
    // Biến cho bộ lọc Ngã rẽ
    static unsigned long time_luot_nga_re = 0;
    static bool dang_luot_nga_re = false;
    static int virtual_error_nga_re = 0;

    // Biến cho bộ lọc Case 1
    static unsigned long time_cho_case_1 = 0;
    static bool dang_cho_case_1 = false;

    // 1. KIỂM TRA NGÃ RẼ TRƯỚC (Ưu tiên cao nhất)
    int buoc_nhay = abs(rawAbsError - previous_rawAbsError);
    if (previous_rawAbsError <= 1 && buoc_nhay >= NGUONG_NGA_RE) {
        if (!dang_luot_nga_re) {
            dang_luot_nga_re = true;
            time_luot_nga_re = millis();
            virtual_error_nga_re = previous_rawAbsError; // Lưu lại số an toàn (0 hoặc 1)
        }
    }

    // 2. KIỂM TRA CASE 1 (Từ 0 sang 1)
    if (rawAbsError == 1 && previous_rawAbsError == 0) {
        dang_cho_case_1 = true;
        time_cho_case_1 = millis();
    }

    // 3. XUẤT ERROR ẢO
    int effective_absError = rawAbsError;

    if (dang_luot_nga_re) {
        if (millis() - time_luot_nga_re < TIME_LUOT_NGA_RE) {
            effective_absError = virtual_error_nga_re; // "Bịt mắt" giữ thẳng lái
        } else {
            dang_luot_nga_re = false;
        }
    } 
    else if (dang_cho_case_1) {
        if (rawAbsError == 1) {
            if (millis() - time_cho_case_1 < TIME_DELAY_CASE_1) {
                effective_absError = 0; // Chờ 200ms mới cho phép bẻ vô lăng
            } else {
                dang_cho_case_1 = false;
            }
        } else {
            dang_cho_case_1 = false; // Văng ra số lớn hơn, hủy bộ đếm lập tức
        }
    }

    previous_rawAbsError = rawAbsError;
    int absError = effective_absError; 
    
    // Phục hồi lại dấu (+/-) cho bộ PID
    float active_error = (steering_error > 0) ? absError : -absError;
    if (steering_error == 0) active_error = 0;
    // =========================================================================

    smoothedError = (active_error * 0.35) + (smoothedError * 0.65); 
    
    float target_Kp, target_Kd;
    int target_offset_raw;
    
    if (absError == 0)      { target_Kp = Kp_123; target_Kd = Kd_123; target_offset_raw = 0; } 
    else if (absError <= 3) { target_Kp = Kp_123; target_Kd = Kd_123; target_offset_raw = BU_ZONE_123; } 
    else if (absError <= 6) { target_Kp = Kp_456; target_Kd = Kd_456; target_offset_raw = BU_ZONE_456; } 
    else                    { target_Kp = Kp_789; target_Kd = Kd_789; target_offset_raw = BU_ZONE_789; }
    
    float speed_ratio = (float)(startSpeed - baseSpeed) / (startSpeed - speed);
    speed_ratio = constrain(speed_ratio, 0.0, 1.0);
    int target_offset = (int)(target_offset_raw * speed_ratio);

    static float current_Kp = Kp_123;
    static float current_Kd = Kd_123;
    static float current_offset = 0.0;
    
    int UPDATE_RATE = 5; 
    if (millis() - last_ramp_time >= UPDATE_RATE) 
    {
        last_ramp_time = millis();
        
        float time_vao = max(TIME_VAO_CUA, 1.0f);
        float time_ra  = max(TIME_RA_CUA,  1.0f);

        float step_offset_vao = ((float)BU_ZONE_789 / time_vao) * UPDATE_RATE;
        float step_offset_ra  = ((float)BU_ZONE_789 / time_ra) * UPDATE_RATE;
        
        float step_Kp_vao = ((Kp_789 - Kp_123) / time_vao) * UPDATE_RATE;
        float step_Kp_ra  = ((Kp_789 - Kp_123) / time_ra) * UPDATE_RATE;

        float step_Kd_vao = ((Kd_789 - Kd_123) / time_vao) * UPDATE_RATE;
        float step_Kd_ra  = ((Kd_789 - Kd_123) / time_ra) * UPDATE_RATE;

        if (current_offset < target_offset) current_offset = min(current_offset + step_offset_vao, (float)target_offset);
        else if (current_offset > target_offset) current_offset = max(current_offset - step_offset_ra, (float)target_offset);

        if (current_Kp < target_Kp) current_Kp = min(current_Kp + step_Kp_vao, target_Kp);
        else if (current_Kp > target_Kp) current_Kp = max(current_Kp - step_Kp_ra, target_Kp);

        if (current_Kd < target_Kd) current_Kd = min(current_Kd + step_Kd_vao, target_Kd);
        else if (current_Kd > target_Kd) current_Kd = max(current_Kd - step_Kd_ra, target_Kd);
    }

    int finalBaseSpeed = baseSpeed + (int)current_offset; 
    finalBaseSpeed = constrain(finalBaseSpeed, speed, startSpeed); 
    
    integral = constrain(integral + smoothedError, -100, 100); 
    float derivative = smoothedError - lastError; 
    
    float rawPid = (current_Kp * smoothedError) + (Ki * integral) + (current_Kd * derivative);
    lastError = smoothedError;
    
    float filteredPid = (rawPid * 0.2) + (lastPidOutput * 0.8); 
    lastPidOutput = filteredPid;
    
    int v_trai = constrain(finalBaseSpeed - (int)filteredPid, speed, startSpeed);
    int v_phai = constrain(finalBaseSpeed + (int)filteredPid, speed, startSpeed); 
    step_dc(true, true, HIGH, LOW, v_phai, v_trai);
}