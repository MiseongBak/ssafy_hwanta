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

import s10p22c110.autodriving.model.Destination;
import s10p22c110.autodriving.service.DestinationService;

@RestController
@RequestMapping("/api/destinations")
public class DestinationController {

    @Autowired
    private DestinationService destinationService;
    
    @PostMapping
    public ResponseEntity<Destination> createDestination(@RequestBody Destination destination) {
        Destination saveDestination = destinationService.saveDestination(destination);
        return ResponseEntity.ok(saveDestination);
    }

    @GetMapping
    public ResponseEntity<List<Destination>> getAllDestinations() {
        List<Destination> destinations = destinationService.findAllDestinations();
        return ResponseEntity.ok(destinations);
    }

    // 특정 id를 가진 Destination 정보 조회
    @GetMapping("/{id}")
    public ResponseEntity<Destination> getDestinationById(@PathVariable Long id) {
        Destination destination = destinationService.findDestinationById(id);
        return ResponseEntity.ok(destination);
    }

    // 특정 ID를 가진 Destination의 lat과 lon 데이터 값을 수정
    @PutMapping("/{id}")
    public ResponseEntity<Destination> updateDestinationLocation(@PathVariable Long id, @RequestBody Map<String, String> location) {
        String lat = location.get("lat");
        String lon = location.get("lon");
        Destination updateDestination = destinationService.updateDestinationLocation(id, lat, lon);
        return ResponseEntity.ok(updateDestination);
    }
}
