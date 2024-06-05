package s10p22c110.autodriving.service;

import java.util.List;
import java.util.Optional;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import s10p22c110.autodriving.model.Patient;
import s10p22c110.autodriving.repository.PatientRepository;

@Service
public class PatientService {

    @Autowired
    private PatientRepository patientRepository;

    public Patient savePatient(Patient patient) {
        if (patient == null) {
            throw new IllegalArgumentException("Input patient cannot be null.");
        }
        return patientRepository.save(patient);
    }

    // 모든 Patient 정보 조회
    public List<Patient> findAllPatients() {
        return patientRepository.findAll();
    }

    // 특정 id를 가진 Patient 정보 조회
    public Patient findPatientById(Long id) {
        if (id == null) {
            throw new IllegalArgumentException("ID cannot be null.");
        }
        Optional<Patient> patient = patientRepository.findById(id);
        return patient.orElse(null);
    }

    // 특정 ID를 가진 Patient의 lat과 lon 데이터 값을 수정하는 메소드
    public Patient updatePatientLocation(Long id, String lat, String lon) {
        if (id == null) {
            throw new IllegalArgumentException("ID cannot be null.");
        }
        Optional<Patient> patientOptional = patientRepository.findById(id);
        if (patientOptional.isPresent()) {
            Patient patient = patientOptional.get();
            patient.setLat(lat);
            patient.setLon(lon);
            return patientRepository.save(patient);
        } else {
            throw new IllegalArgumentException("Patient with ID " + id + " not found.");
        }
    }
}
