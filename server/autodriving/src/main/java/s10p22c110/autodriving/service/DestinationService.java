package s10p22c110.autodriving.service;

import java.util.List;
import java.util.Optional;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import s10p22c110.autodriving.model.Destination;
import s10p22c110.autodriving.repository.DestinationRepository;

@Service
public class DestinationService {

    @Autowired
    private DestinationRepository destinationRepository;

    public Destination saveDestination(Destination destination) {
        if (destination == null) {
            throw new IllegalArgumentException("Input departual cannot be null.");
        }
        return destinationRepository.save(destination);
    }
    
    
    // 모든 Destination 정보 조회
    public List<Destination> findAllDestinations() {
        return destinationRepository.findAll();
    }

    // 특정 id를 가진 Destination 정보 조회
    public Destination findDestinationById(Long id) {
        if (id == null) {
            throw new IllegalArgumentException("ID cannot be null.");
        }
        Optional<Destination> destination = destinationRepository.findById(id);
        return destination.orElse(null);
    }

    // 특정 ID를 가진 Destination lat과 lon 데이터 값을 수정하는 메소드
    public Destination updateDestinationLocation(Long id, String lat, String lon) {
        if (id == null) {
            throw new IllegalArgumentException("ID cannot be null.");
        }
        Optional<Destination> destinationOptional = destinationRepository.findById(id);
        if (destinationOptional.isPresent()) {
            Destination destination = destinationOptional.get();
            destination.setLat(lat);
            destination.setLon(lon);
            return destinationRepository.save(destination);
        } else {
            throw new IllegalArgumentException("Patient with ID " + id + " not found.");
        }
    }
}
