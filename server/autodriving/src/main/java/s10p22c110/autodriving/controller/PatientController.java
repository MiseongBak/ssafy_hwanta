package s10p22c110.autodriving.controller;

import java.util.List;
import java.util.Map;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.PutMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import s10p22c110.autodriving.model.Patient;
import s10p22c110.autodriving.service.PatientService;

@RestController
@RequestMapping("/api/patients")
public class PatientController {

    @Autowired
    private PatientService patientService;

    @PostMapping
    public ResponseEntity<Patient> createDepartual(@RequestBody Patient patient) {
        Patient savedDepartual = patientService.savePatient(patient);
        return ResponseEntity.ok(savedDepartual);
    }

    @GetMapping
    public ResponseEntity<List<Patient>> getAllPatients() {
        List<Patient> patients = patientService.findAllPatients();
        return ResponseEntity.ok(patients);
    }

    // 특정 id를 가진 Patient 정보 조회
    @GetMapping("/{id}")
    public ResponseEntity<Patient> getPatientById(@PathVariable Long id) {
        Patient patient = patientService.findPatientById(id);
        return ResponseEntity.ok(patient);
    }

    // 특정 ID를 가진 Patient의 lat과 lon 데이터 값을 수정
    @PutMapping("/{id}")
    public ResponseEntity<Patient> updatePatientLocation(@PathVariable Long id, @RequestBody Map<String, String> location) {
        String lat = location.get("lat");
        String lon = location.get("lon");
        Patient updatedPatient = patientService.updatePatientLocation(id, lat, lon);
        return ResponseEntity.ok(updatedPatient);
    }
}
