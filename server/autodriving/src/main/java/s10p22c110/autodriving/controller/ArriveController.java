package s10p22c110.autodriving.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import s10p22c110.autodriving.service.ArriveService;

@RestController
@RequestMapping("/api/arrive")
public class ArriveController {

    @Autowired
    private ArriveService arriveService;

    @GetMapping("/{id}")
    public ResponseEntity<Boolean> checkArrival(@PathVariable("id") Long id) {
        boolean isArrived = arriveService.checkArrival(id);
        return ResponseEntity.ok(isArrived);
    }    
}
