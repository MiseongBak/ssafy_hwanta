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

import s10p22c110.autodriving.model.Car;
import s10p22c110.autodriving.service.CarService;

@RestController
@RequestMapping("/api/cars")
public class CarController {
    
    @Autowired
    private CarService carService;

    @PostMapping
    public ResponseEntity<Car> createCar(@RequestBody Car car) {
        Car savedCar = carService.saveCar(car);
        return ResponseEntity.ok(savedCar);
    }

    @GetMapping
    public ResponseEntity<List<Car>> getAllCars() {
        List<Car> cars = carService.findAllCars();
        return ResponseEntity.ok(cars);
    }

    // 특정 id를 가진 Patient 정보 조회
    @GetMapping("/{id}")
    public ResponseEntity<Car> getCarById(@PathVariable Long id) {
        Car car = carService.findCarById(id);
        return ResponseEntity.ok(car);
    }

    // 특정 ID를 가진 Car의 lat과 lon 데이터 값을 수정
    @PutMapping("/{id}")
    public ResponseEntity<Car> updateCarLocation(@PathVariable Long id, @RequestBody Map<String, String> location) {
        String lat = location.get("lat");
        String lon = location.get("lon");
        Car updateCar = carService.updateCarLocation(id, lat, lon);
        return ResponseEntity.ok(updateCar);
    }
}
