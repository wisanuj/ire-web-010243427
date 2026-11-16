# IRE_Web — สื่อการสอนแบบโต้ตอบ วิชา 010243427 วิศวกรรมหุ่นยนต์อุตสาหกรรม

เว็บไซต์การเรียนแบบโต้ตอบสองภาษา (ไทย/อังกฤษ) สำหรับนักศึกษาปริญญาตรี ภาควิชาวิศวกรรมการผลิตและหุ่นยนต์ มจพ.
วิชา **010243427 Industrial Robotics Engineering 3(3-0-6)** ภาคการศึกษา 2/2569 · ตำราหลัก Craig, *Introduction to Robotics: Mechanics and Control* (ใช้สัญลักษณ์ของ Craig ทั้งเว็บ)

Interactive Thai/English teaching site for KMUTNB course 010243427 Industrial Robotics Engineering: real-time 3-D arms you can move, slide pictures woven into each page, and step-by-step solutions.

## เนื้อหาที่มีแล้ว

| บท | หน้า |
|---|---|
| 1 บทนำ | `ch1_1_what_is_robot` หุ่นยนต์คืออะไร · `ch1_2_modeling` แบบจำลองทางคณิตศาสตร์ · `ch1_3_mechanical` หุ่นยนต์ในฐานะอุปกรณ์เชิงกล · `ch1_4_arrangements` โครงสร้างแขนกลที่พบบ่อย |
| 2 การบรรยายตำแหน่งและทิศทาง | `ch2_1_frames_rotation` · `ch2_2_mapping` · `ch2_3_transform_arithmetic` · `ch2_4_orientation` |
| 3 จลนศาสตร์ไปข้างหน้า | `ch3_1_link_parameters` · `ch3_2_link_transform` · `ch3_3_examples` · `ch3_4_dh_conventions` |
| 4 จลนศาสตร์ผกผัน | `ch4_1_solvability` · `ch4_2_algebraic` · `ch4_3_geometric` · `ch4_4_six_axis` |
| 5 จาโคเบียน | `ch5_1_differential_motion` · `ch5_2_direct_derivative` · `ch5_3_basic_jacobian` · `ch5_4_using_jacobian` |
| 6 การส่งต่อความเร็ว | `ch6_1_rigid_body_velocity` · `ch6_2_velocity_propagation` · `ch6_3_jacobian_frames` · `ch6_4_explicit_form` |

บทที่ 7–9 จะเพิ่มทีละบท (แผนที่การเรียนใน `index.html` แสดงเป็น "เร็ว ๆ นี้")

ทุกหน้า: ภาพสไลด์อยู่ในเนื้อเรื่อง · ภาพจำลองแยกทีละโจทย์ เรียงตามลำดับ · ลองตอบก่อน แล้วเปิดเฉลยทีละขั้นด้วยปุ่ม · แขนกลในภาพจำลองบอกพิกัด x, y, z ของปลายแขนตลอดเวลา

## วิธีเปิด

- เปิด `index.html` ในเบราว์เซอร์ หรือเสิร์ฟโฟลเดอร์นี้: `python3 -m http.server 8796` แล้วเปิด `http://localhost:8796/`
- ไม่มีขั้นตอน build ไม่มี framework · ครั้งแรกต้องต่ออินเทอร์เน็ตเพื่อโหลด KaTeX (cdnjs) และฟอนต์
- ภาพสามมิติวาดด้วย WebGL ของเว็บเอง (`assets/gl3d.js`) ถ้าเครื่องไม่มี WebGL จะวาดภาพแบนแทน
- สองภาษา ธีม และความคืบหน้าเก็บใน `localStorage` คีย์ `ire-lang`, `ire-theme`, `ire-visited`

## โครงสร้าง

- `assets/robot.js` คณิตศาสตร์ของหุ่นยนต์ (การแปลงพิกัด เมทริกซ์การหมุน Denavit–Hartenberg จลนศาสตร์) ใช้ได้ทั้งในเบราว์เซอร์และใน deno
- `assets/chN.js` ตัวเลขและเฉลยของบท N · `assets/chN_3d.js` ฉากสามมิติของบท N · `assets/scene3d.js` ตารางฉากและการวางป้ายไม่ให้ทับภาพ
- `assets/shell.js`, `lesson.js`, `ire.css` กรอบของหน้า (สองภาษา ธีม แถบเลื่อน เฉลยทีละขั้น)
- `slides/chN/` ภาพสไลด์ฉบับนักศึกษา (ไม่มีเฉลย)

## การทดสอบ

`tests.html` ตรวจตัวเลขทุกบทกับการคำนวณด้วยมือ ตัวอย่างใน Craig และคำตอบในสไลด์ เปิดแล้วต้องขึ้น **ALL PASS** · ชุดเดียวกันรันด้วย deno ได้:

```
deno run --allow-read assets/run_tests.mjs
```

## ที่มาของภาพ

ภาพทุกภาพวาดหรือเรนเดอร์ขึ้นใหม่ ไม่ได้คัดลอกจากตำรา

- แขน 6 แกนในสไลด์คือ **Yaskawa GP8** ตัวเดียวกับที่ภาควิชามี: เรนเดอร์จากตาข่ายสามมิติของชุด `motoman_gp8_support` ของ ROS-Industrial (BSD licence) ไฟล์ตาข่ายไม่ได้อยู่ในเว็บนี้ มีเฉพาะภาพที่เรนเดอร์แล้ว · ขนาดและช่วงข้อต่อในตาราง Denavit–Hartenberg มาจากแบบจำลองเดียวกันและข้อมูลจำเพาะของ Yaskawa
- แขน SCARA, IRB 360 (เดลตา), Stewart platform และมือจับ เรนเดอร์จากโมเดลที่มากับ CoppeliaSim (Coppelia Robotics, BSD licence)

ผู้สอน: Asst. Prof. Dr. Amornphun Phunopas และ ผศ.ดร.วิษณุ จิตวิริยะ
