package s10p22c110.autodriving.service;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import s10p22c110.autodriving.model.Car;
import s10p22c110.autodriving.model.Patient;
import s10p22c110.autodriving.repository.CarRepository;
import s10p22c110.autodriving.repository.PatientRepository;

@Service
public class ArriveService {

    @Autowired
    private PatientRepository patientRepository;
    @Autowired
    private CarRepository carRepository;

    public boolean checkArrival(Long id) {
        if (id == null) {
            throw new IllegalArgumentException("ID cannot be null.");
        }

        Patient patient = patientRepository.findById(id).orElseThrow();
        Car car = carRepository.findById(id).orElseThrow();

        double patientLat = Double.parseDouble(patient.getLat());
        double patientLon = Double.parseDouble(patient.getLon());
        double carLat = Double.parseDouble(car.getLat());
        double carLon = Double.parseDouble(car.getLon());

        double distance = calculateDistance(patientLat, patientLon, carLat, carLon);

        // 50미터 이내로 가까워졌는지 확인
        return distance <= 50;
    }

    private double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        final int R = 6371; // 지구의 반지름(km)

        double latDistance = Math.toRadians(lat2 - lat1);
        double lonDistance = Math.toRadians(lon2 - lon1);

        double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2) +
                   Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
                   Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);

        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        double distance = R * c * 1000; // 거리를 미터 단위로 변환

        return distance;
    }
}
