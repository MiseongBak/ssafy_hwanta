package s10p22c110.autodriving.service;

import java.util.List;
import java.util.Optional;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import s10p22c110.autodriving.model.Car;
import s10p22c110.autodriving.repository.CarRepository;

@Service
public class CarService {

    @Autowired
    private CarRepository carRepository;

    public Car saveCar(Car car) {
        if (car == null) {
            throw new IllegalArgumentException("Input car cannot be null.");
        }
        return carRepository.save(car);
    }

    // 모든 Car 정보 조회
    public List<Car> findAllCars() {
        return carRepository.findAll();
    }
    
    // 특정 id를 가진 Car 정보 조회
    public Car findCarById(Long id) {
        if (id == null) {
            throw new IllegalArgumentException("ID cannot be null.");
        }
        Optional<Car> car = carRepository.findById(id);
        return car.orElse(null);
    }

    // 특정 ID를 가진 Car의 lat과 lon 데이터 값을 수정하는 메소드
    public Car updateCarLocation(Long id, String lat, String lon) {
        if (id == null) {
            throw new IllegalArgumentException("ID cannot be null.");
        }
        Optional<Car> carOptional = carRepository.findById(id);
        if (carOptional.isPresent()) {
            Car car = carOptional.get();
            car.setLat(lat);
            car.setLon(lon);
            return carRepository.save(car);
        } else {
            throw new IllegalArgumentException("Car with ID " + id + " not found.");
        }
    }
}
